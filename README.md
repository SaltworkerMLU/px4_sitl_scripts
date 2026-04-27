
Configure px4 sitl such that you only need to do "commander takeoff" in sitl terminal
'''
param set COM_RCL_EXCEPT 4      # Ignore RC loss in SITL missions
param set NAV_RCL_ACT 0         # No RC loss failsafe
param set NAV_DLL_ACT 0         # No data link failsafe  
param set COM_ARM_WO_GPS 1      # Allow arming without GPS lock (optional)
param save
'''