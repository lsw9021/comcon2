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
import matplotlib.pylab as pl
from scipy.ndimage.filters import gaussian_filter
parser = argparse.ArgumentParser()
parser.add_argument('--name', required=False, type=str)
parser.add_argument('--config', required=True, type=str)
parser.add_argument('--stride', type=int, default=1)
parser.add_argument('--start', type=int, default=10)
parser.add_argument('--end', type=int, default=30)

parser.add_argument("--checkpoint", type=str, default=None)
# parser.add_argument("--checkpoint", type=list, default=None)
args = parser.parse_args()
file_lists = os.listdir(args.checkpoint)
theta = np.arange(0.0, math.pi*2,0.01)
def func(f, theta):
	n = f.shape[0]
	dtheta = 2*math.pi / n

	i0 = np.floor(theta / dtheta).astype(np.int32)
	i1 = i0+1

	i0 = i0%n
	i1 = i1%n

	r = theta / dtheta - np.floor(theta/dtheta)
	return (1-r)*f[i0] + r*f[i1];
config = load_config(args.config)
plt.clf()
ax = plt.subplot(111, projection='polar')
start = args.start
end = args.end
stride = args.stride
count = 0

for file in file_lists:
	file_path = os.path.join(args.checkpoint+file)
	if file.endswith('.pt'):
		idx = int(file[:-3])
		if (idx % stride != 0) or (idx < start) or (idx>end):
			continue
		count += 1
		print(idx)
colors = pl.cm.jet(np.linspace(0,1,count))
for i in range(count):
	idx = start+stride*i

	file_path = os.path.join(args.checkpoint,str(idx)+'.pt')
	print(file_path)
	state_dict = torch.load(file_path)
	f = state_dict['env_state']['force_function']
	print(idx, f)

	n = f.shape[0]
	r = func(f,theta)

	ax.plot(theta, r, color=colors[i],label=str(idx))
	ax.set_rticks([30.0, 50.0,100.0])  # Less radial ticks
	# ax.set_rticks([30.0, 50.0])  # Less radial ticks
	ax.legend()
ax.grid(True)

plt.show()