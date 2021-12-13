import torch
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional as F

import time
import copy
import numpy as np

import filter
from torchutils import xavier_initializer, convert_to_tensor, convert_to_ndarray


class Discriminator(object):
	def __init__(self, dim_state, device, config):
		dim = config['hidden']
		self.fc = nn.Sequential(
							nn.Linear(dim_state,dim[0]),
							nn.LeakyReLU(0.2,inplace=True),
							nn.Linear(dim[0],dim[1]),
							nn.LeakyReLU(0.2,inplace=True),
							nn.Linear(dim[1],1))
		self.fc[0].apply(lambda y: xavier_initializer(y,0.1))
		self.fc[2].apply(lambda y: xavier_initializer(y,0.1))
		self.fc[4].apply(lambda y: xavier_initializer(y,1.0))

		self.filter = filter.MeanStdRuntimeFilter(dim_state)

		self.w_grad = config['w_grad']
		self.w_reg = config['w_reg']
		self.w_decay = config['w_decay']
		self.r_scale = config['r_scale']
		
		self.grad_clip = config['grad_clip']

		self.optimizer = optim.Adam(self.fc.parameters(), lr=config['lr'])

		self.loss = None
		self.expert_accuracy = None
		self.agent_accuracy = None

		self.device = device
		self.fc = self.fc.to(device)

	def __call__(self, ss1):
		if len(ss1.shape) == 1:
			ss1 = ss1.reshape(1, -1)

		ss1_filtered = self.filter(ss1, update = False)
		ss1_tensor = convert_to_tensor(ss1_filtered, self.device)

		with torch.no_grad():
			d = self.fc(ss1_tensor)

		d = convert_to_ndarray(d)
		d = np.clip(d, -1.0, 1.0)
		d = self.r_scale*(1.0 - 0.25*(d-1)*(d-1))

		return d

	def compute_loss(self, s_expert, s_agent):
		s_expert2 = s_expert.clone().detach()
		s_expert2.requires_grad = True
		
		d_expert = self.fc(s_expert)
		d_expert2 = self.fc(s_expert2)
		d_agent = self.fc(s_agent)

		loss_pos = 0.5 * torch.mean(torch.pow(d_expert - 1.0, 2.0))
		loss_neg = 0.5 * torch.mean(torch.pow(d_agent  + 1.0, 2.0))

		self.expert_accuracy = torch.sum(d_expert)
		self.agent_accuracy = torch.sum(d_agent)

		self.loss = 0.5 * (loss_pos + loss_neg)
		if self.w_decay>0:
			for i in range(len(self.fc)):
				classname = self.fc[i].__class__.__name__
				if classname.find('Linear') != -1:
					v = self.fc[i].weight
					self.loss += 0.5 * self.w_decay * torch.sum(v**2)

		if self.w_reg>0:
			v = self.fc[-1].weight
			self.loss += 0.5 * self.w_reg * torch.sum(v**2)

		if self.w_grad>0:
			grad = torch.autograd.grad(outputs=d_expert2,
										inputs=s_expert2,
										grad_outputs=torch.ones(d_expert2.size()).to(self.device),
										create_graph=True,
										retain_graph=True)[0]

			self.loss += 0.5 * self.w_grad * torch.mean(torch.sum(torch.pow(grad,2.0),axis=-1))

	def backward_and_apply_gradients(self):
		self.optimizer.zero_grad()
		self.loss.backward(retain_graph = True)
		for param in self.fc.parameters():
			if param.grad is not None:
				param.grad.data.clamp_(-self.grad_clip, self.grad_clip)
		self.optimizer.step()
		self.loss = None

	def state_dict(self):
		state = {}
		state['fc'] = self.fc.state_dict()
		state['optimizer'] = self.optimizer.state_dict()
		state['filter'] = self.filter.state_dict()

		return state

	def load_state_dict(self, state):
		self.fc.load_state_dict(state['fc'])
		self.optimizer.load_state_dict(state['optimizer'])
		self.filter.load_state_dict(state['filter'])