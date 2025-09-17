#!/bin/bash

# check if klog-deploy is installed
if ! [ $(which klog-deploy) ]; then
    echo "klog-deploy not found, please install klog-robot package"
    exit 1
fi

# download kinfer model
policy_dir="./policies"
remote_policies="mu:~/kodachrome/policies/"

mkdir -p "$policy_dir"
rsync -aLP --ignore-existing "$remote_policies" "$policy_dir/"

# user select kinfer model
policy="$policy_dir/$(find "$policy_dir" -maxdepth 1 -type f -printf "%T@ %f\n" | sort -nr | cut -d' ' -f2- | fzf --prompt 'Policy to deploy: ' --height 20% --reverse)"

if [ -z $policy ]; then
    echo "No policy selected."
fi

echo "Deploying policy: $policy"



#  set can interfaces
for i in {0..4}
do
    echo "Setting up can$i..."
    sudo ip link set can$i down
    sudo ip link set can$i type can bitrate 1000000
    sudo ip link set can$i txqueuelen 1000
    sudo ip link set can$i up
done

# set max torques
bash ~/kbot_deployment/scripts/reset_max_torques.sh

run_firmware() {
    sudo -E chrt 80 /home/dpsh/miniconda3/envs/jax/bin/python main.py $policy
}


# run
# klog-deploy --no-wait run_firmware "$policy"

run_firmware "$policy"

