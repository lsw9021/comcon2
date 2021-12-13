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
from trainer import *

import argparse

import pycomcon
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.tri as tri
from sklearn.decomposition import PCA
from scipy.ndimage.filters import gaussian_filter
parser = argparse.ArgumentParser()
parser.add_argument('--name', required=False, type=str)
parser.add_argument('--config', required=True, type=str)
parser.add_argument("--checkpoint", type=str, default=None)
args = parser.parse_args()



config = load_config(args.config)
runner = Runner(pycomcon.env, config)
state_dict = torch.load(args.checkpoint)

runner.policy.load_state_dict(state_dict['policy_state_dict'])
runner.disc.load_state_dict(state_dict['discriminator_state_dict'])

s_expert = runner.env.get_state_amp_experts()
s_expert_mean = runner.disc.filter.rs.mean
s_expert_std = runner.disc.filter.rs.std
num_samples = 1000
num_subsamples = 10
s_expert_noise = []
for s in s_expert:
	for i in range(num_subsamples):
		s_expert_noise.append(np.random.normal(s, 0.5*s_expert_std))
s_expert_noise = np.array(s_expert_noise)
s_expert = np.vstack([s_expert,s_expert_noise])
runner.set_num_samples(num_samples)
runner.step()

s_agent = []
s_agent_xxx = []
for epi in runner.disc_episodes:
	s_agent.append(epi['STATES_AGENT'])
	s_agent_xxx.append(epi['XXX'])
s_agent = np.vstack(s_agent)
s_agent_xxx = np.vstack(s_agent_xxx)

# s_expert = s_agent
z = np.hstack([runner.disc(s_expert),runner.disc(s_agent)])
pca = PCA(2)
pca.fit(np.vstack([s_expert, s_agent]))
s_expert = pca.transform(s_expert)
s_agent = pca.transform(s_agent)
s_agent_xxx = pca.transform(s_agent_xxx)

s = np.vstack([s_expert,s_agent])
s_min = np.min(s,axis=0)
s_max = np.max(s,axis=0)

ngrid = 100

xi = np.linspace(s_min[0],s_max[0],ngrid)
yi = np.linspace(s_min[1],s_max[1],ngrid)
triang = tri.Triangulation(s[:,0], s[:,1])

interpolator = tri.LinearTriInterpolator(triang, z)

Xi, Yi = np.meshgrid(xi, yi)
zi = interpolator(Xi, Yi)
zi_mean = zi.filled(fill_value=0.0).mean()
zi = zi.filled(fill_value=zi_mean)
zi = 0.5*gaussian_filter(zi,2.0)
figure = plt.figure()
figure.set_figwidth(1920 / figure.dpi)
figure.set_figheight(1080 / figure.dpi)
# mng = plt.get_current_fig_manager()

plt.clf()
#1
cntr = plt.contourf(xi, yi, zi, levels=15, cmap="jet")
# cntr = plt.contour(xi, yi, zi, levels=15, linewidths=2.0, cmap="jet")

n = s_agent.shape[0]
idx = 0
s_diff = s_agent - s_agent_xxx
for i in range(n):
	if np.linalg.norm(s_diff[i])>1e-6: 
		break
	idx += 1
window = 60
plt.plot(s_agent_xxx[idx:idx+window,0],s_agent_xxx[idx:idx+window,1],c='k',dashes=[2, 2])
plt.plot(s_agent[idx:idx+window,0],s_agent[idx:idx+window,1],c='k')
plt.title('Reward')
plt.colorbar()
plt.show()
# from IPython import embed; embed();exit()
# plt.savefig('savefig_default.png')
# cntr = plt.contourf(xi, yi, zi, levels=15, cmap="jet")
# plt.clabel(cntr, inline=1, fontsize=20)

#2
# plt.clf()
# cntr = plt.contourf(xi, yi, zi, levels=15, cmap="jet")
# n = s_agent.shape[0]
# idices = np.random.randint(n, size=10)
# s_diff = s_agent - s_agent_xxx
# plt.quiver(s_agent_xxx[idices,0],s_agent_xxx[idices,1],s_diff[idices,0],s_diff[idices,1], scale_units='xy', angles='xy', scale=1)
# plt.scatter(s_agent_xxx[idices,0],s_agent_xxx[idices,1],c='k')
# plt.scatter(s_agent[idices,0],s_agent[idices,1],c='b')
# plt.colorbar()
# plt.savefig('savefig_default.png')
# plt.show()
