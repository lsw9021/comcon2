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
from scipy.ndimage.filters import gaussian_filter
parser = argparse.ArgumentParser()
parser.add_argument('--name', required=False, type=str)
parser.add_argument('--config', required=True, type=str)
parser.add_argument("--checkpoint", type=str, default=None)
args = parser.parse_args()

config = load_config(args.config)
state_dict = torch.load(args.checkpoint)

f = state_dict['env_state']['force_function']
def func(f, theta):
	n = f.shape[0]
	dtheta = 2*math.pi / n

	i0 = np.floor(theta / dtheta).astype(np.int32)
	i1 = i0+1

	i0 = i0%n
	i1 = i1%n

	r = theta / dtheta - np.floor(theta/dtheta)
	return (1-r)*f[i0] + r*f[i1];

theta = np.arange(0.0, math.pi*2,0.01)
n = f.shape[0]
r = func(f,theta)

plt.clf()
ax = plt.subplot(111, projection='polar')
ax.plot(theta, r)
#ax.set_rticks([30.0, 50.0,100.0,150.0,200.0])  # Less radial ticks
ax.set_rticks([30.0, 50.0,100.0])  # Less radial ticks
ax.grid(True)
plt.show()