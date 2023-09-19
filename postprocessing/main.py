import os
from fnmatch import fnmatch

import numpy as np
import pandas as pd
import ruamel.yaml as ry

from pCrunch import LoadsAnalysis, PowerProduction, FatigueParams
from pCrunch.io import load_FAST_out
from pCrunch.utility import save_yaml, get_windspeeds, convert_summary_stats

# TODO read in simulation source from command line
# TODO export DELs to .mat

from scipy.io import loadmat

import multiprocessing as mp

SIM_TYPE = 'extreme_k_cases_turbsim'
PROJECT_DIR = "/Users/aoifework/Documents/Research/ipc_tuning/"
DATA_DIR =  os.path.join(PROJECT_DIR, 'sl_outputs')

def valid_op_file(fp):
    # return any([fnmatch(fp, ext) for ext in ["*.outb", "*.out"]])
    pattern = SIM_TYPE + '_' '*.mat'
    return fnmatch(fp, pattern)

if __name__ == '__main__':
    
    ## PROJECT DIRECTORY
    results_dir = os.path.join(PROJECT_DIR, "postprocessing_results")
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
        
    save_results = True
    
    outfiles = [
        os.path.join(DATA_DIR, f) for f in os.listdir(DATA_DIR)
        if valid_op_file(f)
    ]
    
    print(f"Found {len(outfiles)} files.")
    
    ## INTERACTING WITH OUTPUT FILES
    
    # The new framework provides an object oriented framework to interact with
    # output files. The easiest way to use this is to use the 'load_FAST_out' function.
    
    # outputs = load_FAST_out(outfiles[:3])
    outputs = [loadmat(fn) for fn in outfiles]
    
    # An instance of 'OpenFASTBinary' (or 'OpenFASTAscii' if applicable) is created.
    # The instance stores the raw data but also provides many useful methods for
    # interacting with the data:
    
    # print(outputs[0].data)
    # print(outputs[0].time)
    # print(outputs[0].channels)
    # print(outputs[0].maxima)
    # print(outputs[0].stddevs)
    
    # Individual channel time series can also be accessed with dict style indexing:
    outputs[0]["Wind1VelX"]
    
    ## PCRUNCH CONFIGURATION
    
    # Channel magnitudes are defined in a dict:
    magnitude_channels = {
        "RootMc1": ["RootMxc1", "RootMyc1", "RootMzc1"],
        "RootMc2": ["RootMxc2", "RootMyc2", "RootMzc2"],
        "RootMc3": ["RootMxc3", "RootMyc3", "RootMzc3"],
    }
    
    # Define channels (and their fatigue slopes) in a dict:
    fp_nacelle = FatigueParams(lifetime=25.0, slope=4.0)
    fp_tower = FatigueParams(lifetime=25.0, slope=4.0)
    fp_blades = FatigueParams(lifetime=25.0, slope=10.0)
    
    raw_outputs = ['OoPDefl1', 'IPDefl1', 'TwstDefl1', 'RotThrust',
                   'TTDspFA', 'TTDspSS', 'TTDspTwst', # Tower-top / yaw bearing fore-aft/side-to-side/angular torsion deflection
                   'RootMxb1', 'RootMyb1', # Blade 1 edgewise/flapwise moment
                   'RootMyc1', 'RootMzc1', # Blade 1 out - of - plane / pitching moment
                   'LSSGagMya', 'LSSGagMza', # Rotating low - speed shaft bending moment at the shaft's strain gage (about ya/za axis)
                   'YawBrMxp', 'YawBrMyp', 'YawBrMzp', # Nonrotating tower - top / yaw bearing roll / pitch / yaw moment
                   'TwrBsMxt', 'TwrBsMyt', 'TwrBsMzt'# Tower base roll( or side - to - side) / pitching( or fore - aft) / yaw moment
    ]
    
    fatigue_channels = \
        {
            key: fp_blades for key in
            ['RootMc1', 'OoPDefl1', 'IPDefl1', 'TwstDefl1', 'RootMxb1', 'RootMyb1', 'RootMyc1', 'RootMzc1']
         } | {
            key: fp_tower for key in
            ['TTDspFA', 'TTDspSS', 'TTDspTwst', 'TwrBsMxt', 'TwrBsMyt', 'TwrBsMzt']
        } | {
            key: fp_nacelle for key in
            ['RotThrust', 'YawBrMxp', 'YawBrMyp', 'YawBrMzp']
        }
    
    # Define channels to save extreme data in a list:
    channel_extremes = [
        # "RotSpeed",
        # "RotThrust",
        # "RotTorq",
        # "RootMc1",
        # "RootMc2",
        # "RootMc3",
    ]
    
    ## RUN PCRUNCH
    
    # The API has changed and is in more of an object oriented framework.
    la = LoadsAnalysis(
        outfiles[:5],  # The primary input is a list of output files
        magnitude_channels=magnitude_channels,  # All of the following inputs are optional
        fatigue_channels=fatigue_channels,  #
        extreme_channels=channel_extremes,  #
        trim_data=(0,),  # If 'trim_data' is passed, all input files will
    )  # be trimmed to (tmin, tmax(optional))
    
    la.process_outputs(cores=mp.cpu_count())  # Once LoadsAnalysis is configured, process outputs with
    # `process_outputs`. `cores` is optional but will trigger parallel processing if configured
    
    ## OUTPUTS
    # The summary stats per each file are here:
    la.summary_stats
    
    # These are indexable by channel, stat, file:
    la.summary_stats["RootMc1"]
    la.summary_stats[("RootMc1", 'min')]
    la.summary_stats.loc[outfiles[0]]
    
    # Damage equivalent loads are found here:
    la.DELs
    
    # Extreme events:
    la.extreme_events