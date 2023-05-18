#!/bin/bash
#SBATCH --mem=4GB
#SBATCH --time=30:00
#SBATCH --cpus-per-task=1
#SBATCH --output=output.out
#SBATCH --error=error.err

module purge
module load matlab/2021a



cd /scratch/$USER/Project #change this to path of your code folder

echo "$SLURM_ARRAY_TASK_ID"


matlab -nodisplay -r "run_sim,exit"
