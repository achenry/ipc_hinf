#!/bin/bash

#SBATCH --nodes=1
#SBATCH --time=00:15:00
#SBATCH --ntasks=2
#SBATCH --partition=atesting
#SBATCH --qos=testing
#SBATCH --job-name=nonlin_sim_debug
#SBATCH --output=nonlin_sim_debug.%j.out

module purge
module load matlab/R2022b
cd /projects/aohe7145/projects/ipc_tuning/code
matlab -nodisplay -nosplash -nodesktop -r "run('nonlinear_analysis.m');exit;" | tail -n +11
