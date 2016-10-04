For a new computer:
* `mkdir ~/.ssh`
* `cd ~/.ssh/`
* `chmod 700 ~/.ssh`
* `ssh-keygen -t rsa`
* Enter `apc_id_rsa`
* No pass phrase
* `ssh -oHostKeyAlgorithms='ssh-rsa' dockpc@dockpc-nuc`
* Say YES
* Exit the NUC
* `ssh-copy-id dockpc@dockpc-nuc`

eval "$(ssh-agent -s)"
ssh-add ~/.ssh/apc_id_dsa


Enter computer password

Move the left arm out of the way of the kinects sight of the shelf

roslaunch state_machine baxter_perception.launch

Launch the joint trajectory action server on another computer!!!

roslaunch state_machine baxter_motion_planning.launch

Clear the scene to remove the shelf

roslaunch state_machine baxter_state_machine.launch
