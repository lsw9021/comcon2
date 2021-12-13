import torch
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional as F

import numpy as np
import scipy

import importlib.util
def xavier_initializer(m, gain):
	classname = m.__class__.__name__
	if classname.find('Linear') != -1:
		torch.nn.init.xavier_uniform_(m.weight, gain)
		m.bias.data.zero_()


def discount(x, gamma):
	return scipy.signal.lfilter([1],[1, -gamma], x[::-1], axis=0)[::-1]
	
def convert_to_tensor(arr, device):
	if torch.is_tensor(arr):
		return arr.to(device)
	tensor = torch.from_numpy(np.asarray(arr))
	if tensor.dtype == torch.double:
		tensor = tensor.float()
	return tensor.to(device)

def convert_to_ndarray(arr):
	if isinstance(arr, np.ndarray):
		if arr.dtype == np.float64:
			return arr.astype(np.float32)
		return arr
	return arr.cpu().detach().numpy().squeeze()

def load_config(path):
	spec = importlib.util.spec_from_file_location("config", path)
	spec_module = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(spec_module)
	spec = spec_module

	return spec.config