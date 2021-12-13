#!/bin/bash
#SBATCH --job-name=comcon_push_recovery
#SBATCH --ntasks=64
#SBATCH --nodes=1

module load cuda/cuda-11.0

source ~/venv/bin/activate

mpirun python3 train.py --config config.py
