# multihop_routing_VANETs
NYU ECE-7363 course project


## How to run the code on NYU HPC?

Transfer the code to your HPC login node using SCP. It is recommended to save your files in the scratch folder.

SSH into your HPC node: 

```
ssh <your_net_id>@greene.hpc.nyu.edu 
```

Navigate to the code directory, for example:
```
cd scratch/as12738/multihop_routing_VANETs
```
Configure the simulation parameters in run_sim.m 

Make sure that the path in submit.sbatch matches the path of your code in your HPC login node

Then run:

```
sbatch --array=1-100 submit.sbatch
```

This command runs 100 simulations in parallel. It should take 10-15 minutes to run once you get the HPC resources. The sim_time parameter can be adjusted (default is 100 seconds). You can also run a single simulation on your local desktop/PC. 

Data will be collected and stored in a folder named data. Transfer this data back to your laptop using SCP and then plot the relevant metrics using 'data_process.m'


## Important functions
run_sim.m : Main simulation file. Creates a deployment scenario for each value of ndoe density and also generates traffic. Then feeds them into the main network simulation (vanet_broadcast_sim.m). Saves results to local folders.

vanet_broadcast_sim.m : Runs the simulation for a single deployment scenario, discrete event simulator modeling the broadcast transmissions of safety messages across a vehicular highway network. Implements the IFP protocol.


