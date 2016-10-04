eval "$(ssh-agent -s)"
ssh-add ~/.ssh/apc_id_dsa

Enter computer password

Make sure every other node is off i.e. baxter_perception launch

roslaunch apc_3d_vision calibrate_shelf.launch
