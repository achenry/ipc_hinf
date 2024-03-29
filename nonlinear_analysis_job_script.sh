#!/bin/bash

#SBATCH --nodes=2
#SBATCH --time=05:00:00
#SBATCH --ntasks=96
#SBATCH --partition=amilan
#SBATCH --job-name=nonlin_sim
#SBATCH --output=nonlin_sim.%j.out

module purge
module load matlab/R2022b
cd /projects/aohe7145/projects/ipc_tuning/code
matlab -nodisplay -nosplash -nodesktop -r "run('nonlinear_analysis.m'); exit;" | tail -n +11
