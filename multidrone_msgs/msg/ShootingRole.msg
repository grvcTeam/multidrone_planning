ShootingType shooting_type                      # type: STATIC, FLY_THROUGH, ESTABLISH, CHASE, LEAD, FLYBY, LATERAL, ELEVATOR, ORBIT.
FramingType framing_type                        # type: ELS, VLS, LS, MS, MCU, CU, OTS.
ShootingParameter[] shooting_parameters         # vector of pair<string,float>, which strings are: z_0, x_s, x_e, z_s, z_e, y_0, r_0, azimuth_s, azimuth_speed.

TargetIdentifierType target_identifier_type     # where to point the gimbal, type: NONE, VIRTUAL, GPS, VISUAL, STATIC, VISUAL_GPS.
ShootingParameter[] target_parameters           # vector of <string,float> pairs, which strings are: if GPS: ID ; if VISUAL: - ; if VISUAL_GPS: ID ; if VIRTUAL: - ; if NONE: {pan_s, tilt_s, pan_e, tilt_e}; if STATIC: {Latitude, Longitude, x, y, Altitude}
