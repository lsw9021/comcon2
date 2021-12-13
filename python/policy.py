import torch
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional as F

import copy
import numpy as np
import scipy.signal

import time
import filter
from torchutils import xavier_initializer, discount, convert_to_tensor, convert_to_ndarray


class Policy(object):
	def __init__(self, dim_state, dim_action, device, config):
		if isinstance(device, str):
			self.device = torch.device(device)
		else:
			self.device = device
		dim = config['hidden']
		self.value_fc = nn.Sequential(
						nn.Linear(dim_state,dim[0]),
						nn.LeakyReLU(0.2,inplace=True),
						nn.Linear(dim[0],dim[1]),
						nn.LeakyReLU(0.2,inplace=True),
						nn.Linear(dim[1],1))

		self.policy_fc = nn.Sequential(
						nn.Linear(dim_state,dim[0]),
						nn.LeakyReLU(0.2,inplace=True),
						nn.Linear(dim[0],dim[1]),
						nn.LeakyReLU(0.2,inplace=True),
						nn.Linear(dim[1],dim_action))

		self.value_fc[0].apply(lambda y: xavier_initializer(y,0.1))
		self.value_fc[2].apply(lambda y: xavier_initializer(y,0.1))
		self.value_fc[4].apply(lambda y: xavier_initializer(y,0.01))

		self.policy_fc[0].apply(lambda y: xavier_initializer(y,0.1))
		self.policy_fc[2].apply(lambda y: xavier_initializer(y,0.1))
		self.policy_fc[4].apply(lambda y: xavier_initializer(y,0.01))

		self.filter = filter.MeanStdRuntimeFilter(dim_state)
		self.distribution = torch.distributions.normal.Normal
		self.gamma = config['gamma']
		self.lb = config['lb']
		self.std = torch.tensor([config['std']]*dim_action).to(self.device)

		self.policy_clip = config['policy_clip']
		self.value_clip = config['value_clip']
		self.grad_clip = config['grad_clip']

		self.w_kl = config['kl']
		self.w_entropy = config['entropy']

		self.value_optimizer = optim.Adam(self.value_fc.parameters(), lr=config['value_lr'])
		self.policy_optimizer = optim.Adam(self.policy_fc.parameters(), lr=config['policy_lr'])

		self.value_loss = None
		self.policy_loss = None

		self.value_fc = self.value_fc.to(self.device)
		self.policy_fc = self.policy_fc.to(self.device)

	def __call__(self, state, explore=True):
		if len(state.shape) == 1:
			state = state.reshape(1, -1)

		state_filtered = self.filter(state, update=False)
		state_tensor = convert_to_tensor(state_filtered ,self.device)

		with torch.no_grad():
			vf_pred = self.value_fc(state_tensor)
			mean = self.policy_fc(state_tensor)
			
			action_dist = self.distribution(mean, self.std)
			action = action_dist.sample()
			log_prob = action_dist.log_prob(action).sum(-1)
			if explore == False:
				action = action_dist.loc

		action = convert_to_ndarray(action)
		log_prob = convert_to_ndarray(log_prob)
		vf_pred = convert_to_ndarray(vf_pred)

		return action, log_prob, vf_pred

	def compute_ppo_td_gae(self, episode):
		ret = {}
		retsize = len(episode['ACTIONS'])
		vpred_t = np.concatenate([episode['VF_PREDS'], np.array([0.0])])
		delta_t = episode['REWARDS'] + self.gamma * vpred_t[1:] - vpred_t[:-1]

		ret['ADVANTAGES'] = discount(delta_t, self.gamma * self.lb)
		ret['VALUE_TARGETS'] = ret['ADVANTAGES'] + episode['VF_PREDS']

		return ret

	def compute_loss(self, states, actions, vf_preds, log_probs, advantages, value_targets):
		curr_vf_pred = self.value_fc(states)
		curr_mean = self.policy_fc(states)

		curr_action_dist = self.distribution(curr_mean, self.std)

		logp_ratio = torch.exp(
			curr_action_dist.log_prob(actions).sum(-1) - log_probs)

		surrogate_loss = torch.min(
			advantages * logp_ratio,
			advantages * torch.clamp(logp_ratio, 1.0 - self.policy_clip, 1.0 + self.policy_clip))

		entropy_loss = self.w_entropy * curr_action_dist.entropy().sum(-1)

		curr_vf_pred = curr_vf_pred.reshape(-1)
		vf_loss1 = torch.pow(curr_vf_pred - value_targets, 2.0)
		vf_clipped = vf_preds + torch.clamp(curr_vf_pred - vf_preds, -self.value_clip, self.value_clip)

		vf_loss2 = torch.pow(vf_clipped - value_targets, 2.0)
		vf_loss = torch.max(vf_loss1, vf_loss2)

		self.policy_loss = - torch.mean(surrogate_loss) - torch.mean(entropy_loss)
		self.value_loss = torch.mean(vf_loss)

	def backward_and_apply_gradients(self):
		self.value_optimizer.zero_grad()
		self.value_loss.backward(retain_graph=True)
		for param in self.value_fc.parameters():
			if param.grad is not None:
				param.grad.data.clamp_(-self.grad_clip,self.grad_clip)
		self.value_optimizer.step()
		self.value_loss = None

		self.policy_optimizer.zero_grad()
		self.policy_loss.backward(retain_graph=True)
		for param in self.policy_fc.parameters():
			if param.grad is not None:
				param.grad.data.clamp_(-self.grad_clip,self.grad_clip)
		self.policy_optimizer.step()
		self.policy_loss = None

	def state_dict(self):
		state = {}
		state['fc'] = [self.value_fc.state_dict(),self.policy_fc.state_dict()]
		state['optimizer'] = [self.value_optimizer.state_dict(),self.policy_optimizer.state_dict()]
		state['filter'] = self.filter.state_dict()

		return state

	def load_state_dict(self, state):
		self.value_fc.load_state_dict(state['fc'][0])
		self.policy_fc.load_state_dict(state['fc'][1])
		self.value_optimizer.load_state_dict(state['optimizer'][0])
		self.policy_optimizer.load_state_dict(state['optimizer'][1])
		self.filter.load_state_dict(state['filter'])
