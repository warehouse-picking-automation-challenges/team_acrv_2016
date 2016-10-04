# Change Arms

1. apc_state_machine/state_machine/parameters/global.yaml:  
`which_arm: 'left_arm'` or `which_arm: 'right_arm'`
1. Toggle apc_state_machine/state_machine/launch/baxter_motion_planning.launch:  
`<arg name="left_arm" value="true" />`
1. Toggle apc_state_machine/state_machine/launch/baxter_perception.launch:  
`<arg name="left_arm" value="true" />`
1. Change tote position in apc_state_machine/state_machine/parameters/tote_information.json, comment/uncomment or modify as necessary.
1. Change all move_groups from left to right (or vice versa) in (should be handled better):
  * apc_state_machine/mission_plan/mission_descriptions/pre_selected_object_move_groups.json
  * apc_state_machine/mission_plan/mission_descriptions/pre_selected_object_move_groups_stow.json
