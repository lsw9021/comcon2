import time
import math

import torch
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional as F

import numpy as np

import collections
from collections import namedtuple

import pickle
import os

import policy as ppo
import discriminator

from torch.utils.tensorboard import SummaryWriter
from mpi_utils import get_num_procs, get_proc_rank, is_root_proc, is_root2_proc,get_root_proc, get_root2_proc, broadcast, gather, scatter, send, recv

from torchutils import convert_to_tensor, convert_to_ndarray, load_config
# Sample = namedtuple('Sample',('s', 'a', 'rg', 's_amp', 'vf_pred', 'log_prob'))
Sample = namedtuple('Sample',('s', 'a', 'rg', 's_amp', 'vf_pred', 'log_prob'))



class Runner(object):
	def __init__(self, env_cls, config):
		self.env = env_cls()

		self.policy = self.create_policy(config['policy'],torch.device('cpu'))
		self.disc = self.create_disc(config['disc'],torch.device('cpu'))

		self.num_samples = None

		self.env.reset()
		
		self.episode_buffers = []
		self.episode_buffers.append([])

		self.state = self.env.get_state()
	def set_num_samples(self, num_samples):
		self.num_samples = num_samples

	def create_policy(self, config, device):
		return ppo.Policy(self.env.get_dim_state(), self.env.get_dim_action(), device, config)

	def create_disc(self, config, device):
		return discriminator.Discriminator(self.env.get_dim_state_amp(), device, config)

	def step(self):
		self.generate_episodes()
		self.postprocess_episodes()

	def generate_episodes(self):
		for j in range(self.num_samples):
			a, lp, vf = self.policy(self.state)
			self.env.step(a)

			s_amp = self.env.get_state_amp()
			eoe = self.env.eoe()
			rg = self.env.get_reward_position()

			self.episode_buffers[-1].append(Sample(self.state, a, rg, s_amp, vf, lp))
			self.state = self.env.get_state()
			if eoe:
				if len(self.episode_buffers[-1]) != 0:
					self.episode_buffers.append([])
				self.env.reset()
				self.state = self.env.get_state()

	def postprocess_episodes(self):
		self.policy_episodes = []
		self.disc_episodes = []

		for epi in self.episode_buffers[:-1]:
			s, a, rg, s_amp,  v, l = Sample(*zip(*epi))
			s_amp = np.vstack(s_amp)
			r = self.disc(s_amp)
			rg = np.array(rg)
			r = r*rg

			policy_epi = {}
			policy_epi['STATES'] = np.vstack(s)
			policy_epi['ACTIONS'] = np.vstack(a)
			policy_epi['REWARD_GOALS'] = rg.reshape(-1)
			policy_epi['REWARDS'] = r.reshape(-1)
			policy_epi['VF_PREDS'] = np.vstack(v).reshape(-1)
			policy_epi['LOG_PROBS'] = np.vstack(l).reshape(-1)

			td_gae = self.policy.compute_ppo_td_gae(policy_epi)

			for key, item in td_gae.items():
				policy_epi[key] = item
			self.policy_episodes.append(policy_epi)

			disc_epi = {}
			disc_epi['STATES_AGENT'] = s_amp
			self.disc_episodes.append(disc_epi)

		self.episode_buffers = self.episode_buffers[-1:]

	def compute_action(self, state):
		a, lp, vf = self.policy(state, explore=False)
		return a

	def compute_reward(self, state):
		d = self.disc(state)
		return d

class Trainer(object):
	def __init__(self, env_cls, config, path):
		self.runner = Runner(env_cls, config)

		self.path = path
		self.device = torch.device('cuda')
		if is_root_proc():
			self.policy_loc = self.runner.create_policy(config['policy'], self.device)
			self.num_sgd_iter = config['num_sgd_iter']
			self.sgd_minibatch_size = config['sgd_minibatch_size']

		elif is_root2_proc():
			self.disc_loc = self.runner.create_disc(config['disc'], self.device)
			self.num_sgd_iter = config['num_disc_sgd_iter']
			self.sgd_minibatch_size = config['disc_sgd_minibatch_size']

		self.num_envs = get_num_procs()
		self.state_experts = self.runner.env.get_state_amp_experts()
		self.runner.set_num_samples(config['sample_size']//self.num_envs)
		self.save_frequency = config['save_frequency']

		if is_root_proc():
			self.state_dict = {}
			self.state_dict['elapsed_time'] = 0.0
			self.state_dict['num_iterations_so_far'] = 0
			self.state_dict['num_samples_so_far'] = 0

			self.create_summary_writer(path)

	def create_summary_writer(self, path):
		self.writer = SummaryWriter(path)

	def step(self):
		log = {}
		if is_root_proc():
			self.tic = time.time()

		self.sync()
		self.runner.step()
		self.gather()
		if is_root_proc():
			samples = self.concat(self.policy_episodes)

			log_policy = self.optimize_policy(samples)
			log.update(log_policy)

		if is_root2_proc():
			samples = self.concat(self.disc_episodes)
			log_disc = self.optimize_disc(samples)
			log.update(log_disc)

		logs = gather(log, root=get_root_proc())
		
		if is_root_proc():
			log = {}
			for l in logs:
				log.update(l)
			self.print_log(log)
			self.save()

	def sync(self):
		if is_root_proc():
			state_dict = self.policy_loc.state_dict()
			self.runner.policy.load_state_dict(state_dict)
		if is_root2_proc():
			state_dict = self.disc_loc.state_dict()
			self.runner.disc.load_state_dict(state_dict)

		self.runner.policy = broadcast(self.runner.policy, root=get_root_proc())
		self.runner.disc = broadcast(self.runner.disc, root=get_root2_proc())

	def gather(self):
		self.policy_episodes = gather(self.runner.policy_episodes, root=get_root_proc())
		self.disc_episodes = gather(self.runner.disc_episodes, root=get_root2_proc())

	def concat(self, episodes):
		samples = []
		for epis in episodes:
			for epi in epis:
				samples.append(epi)
		if len(samples) == 0:
			return None

		ret = {}
		for key, item in samples[0].items():
			ret[key] = []

		for sample in samples:
			for key, item in sample.items():
				ret[key].append(item)

		for key in ret.keys():
			ret[key] = np.concatenate(ret[key])

		ret['NUM_EPISODES'] = len(samples)
		return ret

	def optimize_policy(self, samples):
		
		if samples == None:
			log = {}
			toc = time.time()
			self.state_dict['elapsed_time'] += toc - self.tic
			log['mean_episode_len'] = 0.0
			log['mean_episode_reward'] = 0.0
			log['mean_episode_reward_goal'] = 0.0

			return log
		n = len(samples['STATES'])
		''' Policy '''
		samples['ADVANTAGES'] = (samples['ADVANTAGES'] - samples['ADVANTAGES'].mean())/(1e-4 + samples['ADVANTAGES'].std())
		samples['STATES'] = self.policy_loc.filter(samples['STATES'])

		samples['STATES'] = convert_to_tensor(samples['STATES'], self.device)
		samples['ACTIONS'] = convert_to_tensor(samples['ACTIONS'], self.device)
		samples['VF_PREDS'] = convert_to_tensor(samples['VF_PREDS'], self.device)
		samples['LOG_PROBS'] = convert_to_tensor(samples['LOG_PROBS'], self.device)
		samples['ADVANTAGES'] = convert_to_tensor(samples['ADVANTAGES'], self.device)
		samples['VALUE_TARGETS'] = convert_to_tensor(samples['VALUE_TARGETS'], self.device)

		for _ in range(self.num_sgd_iter):
			minibatches = self.generate_shuffle_indices(n, self.sgd_minibatch_size)
			for minibatch in minibatches:
				states = samples['STATES'][minibatch]
				actions = samples['ACTIONS'][minibatch]
				vf_preds = samples['VF_PREDS'][minibatch]
				log_probs = samples['LOG_PROBS'][minibatch]
				advantages = samples['ADVANTAGES'][minibatch]
				value_targets = samples['VALUE_TARGETS'][minibatch]

				self.policy_loc.compute_loss(states, actions, vf_preds, log_probs, advantages, value_targets)
				self.policy_loc.backward_and_apply_gradients()

		self.state_dict['num_iterations_so_far'] += 1
		self.state_dict['num_samples_so_far'] += len(samples['REWARDS'])

		log = {}
		toc = time.time()
		self.state_dict['elapsed_time'] += toc - self.tic
		log['mean_episode_len'] = len(samples['REWARDS'])/samples['NUM_EPISODES']
		log['mean_episode_reward'] = np.mean(samples['REWARDS'])
		log['mean_episode_reward_goal'] = np.mean(samples['REWARD_GOALS'])
		return log
	def optimize_disc(self, samples):
		if samples == None:
			log = {}

			log['disc_loss'] = 0.0
			log['expert_accuracy'] = 0.0
			log['agent_accuracy'] = 0.0
			
			return log
		n = len(samples['STATES_AGENT'])

		samples['STATES_EXPERT'] = self.sample_state_experts(n)
		''' Discriminator '''
		state = self.disc_loc.filter(np.vstack([samples['STATES_EXPERT'], samples['STATES_AGENT']]))

		samples['STATES_EXPERT'] = state[:n]
		samples['STATES_AGENT'] = state[n:]

		samples['STATES_EXPERT'] = convert_to_tensor(samples['STATES_EXPERT'], self.device)
		samples['STATES_AGENT'] = convert_to_tensor(samples['STATES_AGENT'], self.device)
		
		disc_loss = 0.0
		disc_grad_loss = 0.0
		expert_accuracy = 0.0
		agent_accuracy = 0.0

		for _ in range(self.num_sgd_iter):
			minibatches = self.generate_shuffle_indices(n, self.sgd_minibatch_size)
			for minibatch in minibatches:
				states_expert = samples['STATES_EXPERT'][minibatch]
				states_agent = samples['STATES_AGENT'][minibatch]

				self.disc_loc.compute_loss(states_expert, states_agent)
				disc_loss += self.disc_loc.loss.detach()
				expert_accuracy += self.disc_loc.expert_accuracy.detach()
				agent_accuracy += self.disc_loc.agent_accuracy.detach()
				self.disc_loc.backward_and_apply_gradients()
		
		log = {}
		log['disc_loss'] = disc_loss.cpu().numpy()/len(minibatches)
		log['expert_accuracy'] = expert_accuracy.cpu().numpy()/n/self.num_sgd_iter
		log['agent_accuracy'] = agent_accuracy.cpu().numpy()/n/self.num_sgd_iter

		return log

	def generate_shuffle_indices(self, batch_size, minibatch_size):
		n = batch_size
		m = minibatch_size
		p = np.random.permutation(n)

		r = m - n%m
		if r>0:
			p = np.hstack([p,np.random.randint(0,n,r)])

		p = p.reshape(-1,m)
		return p

	def sample_state_experts(self, n):
		m = len(self.state_experts)
		return self.state_experts[np.random.randint(0, m, n)]

	def print_log(self, log):
		def time_to_hms(t):
			h = int((t)//3600.0)
			m = int((t)//60.0)
			s = int((t))
			m = m - h*60
			s = t
			s = s - h*3600 - m*60
			return h,m,s
		
		h,m,s=time_to_hms(self.state_dict['elapsed_time'])
		end = '\n'
		print('# {}, {}h:{}m:{:.1f}s'.format(self.state_dict['num_iterations_so_far'],h,m,s),end=end)
		print('policy   len : {:.1f}, rew : {:.3f}, rew_goal : {:.3f}, samples : {:,}'.format(log['mean_episode_len'],
																						log['mean_episode_reward'],
																						log['mean_episode_reward_goal'],
																						self.state_dict['num_samples_so_far']))
		print('discriminator loss : {:.3f} acc_expert : {:.3f}, acc_agent : {:.3f}'.format(log['disc_loss'],
																						log['expert_accuracy'],
																						log['agent_accuracy']))
		print('')
		self.writer.add_scalar('policy/episode_len',log['mean_episode_len'],
				self.state_dict['num_samples_so_far'])

		self.writer.add_scalar('policy/reward_mean',log['mean_episode_reward'],
			self.state_dict['num_samples_so_far'])

		self.writer.add_scalar('policy/reward_mean_goal',log['mean_episode_reward_goal'],
			self.state_dict['num_samples_so_far'])

		self.writer.add_scalar('discriminator/loss',log['disc_loss'],
			self.state_dict['num_samples_so_far'])

		self.writer.add_scalar('discriminator/expert_accuracy',log['expert_accuracy'],
			self.state_dict['num_samples_so_far'])

		self.writer.add_scalar('discriminator/agent_accuracy',log['agent_accuracy'],
			self.state_dict['num_samples_so_far'])

	def save(self):
		cond0 = self.state_dict['num_iterations_so_far'] % self.save_frequency[0] == 0
		cond1 = self.state_dict['num_iterations_so_far'] % self.save_frequency[1] == 0
		if cond0:
			state = {}
			state['policy_state_dict'] = self.runner.policy.state_dict()
			state['discriminator_state_dict'] = self.runner.disc.state_dict()

			state.update(self.state_dict)
			path = os.path.split(self.path)[0]
			torch.save(state, os.path.join(path,'current.pt'))
			print('save at {}'.format(os.path.join(path,'current.pt')))
		if cond1:
			state = {}
			state['policy_state_dict'] = self.runner.policy.state_dict()
			state['discriminator_state_dict'] = self.runner.disc.state_dict()

			state.update(self.state_dict)

			torch.save(state, os.path.join(self.path,str(math.floor(self.state_dict['num_samples_so_far']/1e6))+'.pt'))
			print('save at {}'.format(os.path.join(self.path,str(math.floor(self.state_dict['num_samples_so_far']/1e6))+'.pt')))
			
	def load(self, path):
		if is_root_proc():
			print(f'load {path}')
			state = torch.load(path)
			self.policy_loc.load_state_dict(state['policy_state_dict'])
			for key in self.state_dict.keys():
				self.state_dict[key] = state[key]
		elif is_root2_proc():
			state = torch.load(path)
			self.disc_loc.load_state_dict(state['discriminator_state_dict'])

import pycomcon
def build_runner(config):
	config = load_config(config)
	runner = Runner(pycomcon.env, config)
	return runner

def load_runner(runner, checkpoint):
	state_dict = torch.load(checkpoint)

	runner.policy.load_state_dict(state_dict['policy_state_dict'])
	runner.disc.load_state_dict(state_dict['discriminator_state_dict'])
