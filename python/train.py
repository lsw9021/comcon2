import argparse
import datetime
import os
import time

import trainer as Trainer

from mpi4py import MPI
import pycomcon

from torch.utils.tensorboard import SummaryWriter


from torchutils import load_config

def define_save_path(path, name):
	dyear = datetime.datetime.now().year
	dmonth = datetime.datetime.now().month
	dday = datetime.datetime.now().day
	dhour = datetime.datetime.now().hour
	dminute = datetime.datetime.now().minute

	date_string = f'{dyear}-{dmonth}-{dday}-{dhour}-{dminute}'
	if name is not None:
		savepath = os.path.join(config['save_path'],name)
	else:
		savepath = os.path.join(config['save_path'],date_string)

	return savepath

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('--name', required=False, type=str)
	parser.add_argument('--config', required=True, type=str)
	parser.add_argument("--checkpoint", type=str, default=None)
	args = parser.parse_args()
	
	config = load_config(args.config)
	save_path = define_save_path(config['save_path'], args.name)

	trainer = Trainer.Trainer(pycomcon.env, config, save_path)
	if args.checkpoint is not None:
		trainer.load(args.checkpoint)
	done = False
	try:
		while not done:
			trainer.step()
	except KeyboardInterrupt:
		print('abort mpi')
		MPI.COMM_WORLD.Abort(1)
