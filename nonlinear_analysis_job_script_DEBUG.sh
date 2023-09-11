#!bin/bash
#SBATCH --nodes=1
#SBATCH --time=00:10:00
#SBATCH --ntasks=2
#SBATCH --job-name=nonlin_sim_debug
#SBATCH --output=nonlin_sim_debug.%j.out

module purge
module load matlab
cd /projects/aohe7145/projects/ipc_tuning/code
matlab ./nonlinear_analysis.m
