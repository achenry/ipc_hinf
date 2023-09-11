# scp -r /Users/aoifework/Documents/Research/ipc_tuning aohe7145@login.rc.colorado.edu:/projects/aohe7145/projects/ipc_tuning
rsync -av --exclude 'nonlinear_simulations' --exclude 'nonlinear simulations' --exclude 'simulations' /Users/aoifework/Documents/Research/ipc_tuning aohe7145@login.rc.colorado.edu:/projects/aohe7145/projects/ipc_tuning

scp -r /Users/aoifework/Documents/toolboxes aohe7145@login.rc.colorado.edu:/projects/aohe7145/toolboxes
scp -r /Users/aoifework/Documents/usflowt_src/ aohe7145@login.rc.colorado.edu:/projects/aohe7145/usflowt_src