#!bin/bash
#SBATCH --nodes=2
#SBATCH --time=05:00:00
#SBATCH --ntasks=96
#SBATCH --partition=amem1
#SBATCH --job-name=nonlin_sim
#SBATCH --output=nonlin_sim.%j.out

module purge
module load matlab
cd /projects/aohe7145/projects/ipc_tuning/code
matlab ./nonlinear_analysis.m
