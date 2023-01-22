L_m = 0.2
Dtheta1theta2 = 0.35
Dtheta2theta3 = 0.45
Dtheta3theta4 = 0.15
massbox = 5
density = 2700
outer = 0.051**2
inner = 0.044**2
pi = 22/7
gravity = 9.8

masslink5 = density * L_m * ((pi * outer ) - (pi * inner))
masstheta4 = 0.24
masstheta3 = 0.24
masstheta2 = 0.24

masslinktheta3theta4 = density * Dtheta3theta4 * ((pi * outer ) - (pi * inner))
masslinktheta2theta3 = density * Dtheta2theta3 * ((pi * outer ) - (pi * inner))
masslinktheta1theta2 = density * Dtheta1theta2 * ((pi * outer ) - (pi * inner))

torque4 = (massbox + masslink5) * gravity * L_m/2 * 2

torque3 = ((massbox+masslink5) * (Dtheta3theta4 + L_m/2) * gravity) + (masstheta4 * gravity * Dtheta3theta4) + (masslinktheta3theta4 * gravity * (Dtheta3theta4/2)) 

torque2 = ((massbox + masslink5) * (Dtheta2theta3 + Dtheta3theta4 + L_m/2) * gravity) + (masstheta3 * Dtheta2theta3 * gravity) + (masslinktheta2theta3 * gravity * Dtheta2theta3/2) + (masstheta4 * gravity * (Dtheta2theta3 + Dtheta3theta4)) + (masslinktheta3theta4 * gravity * (Dtheta2theta3 + Dtheta3theta4/2))

torque1 = ((massbox + masslink5) * (Dtheta1theta2 + Dtheta2theta3 + Dtheta3theta4 +L_m/2) * gravity) + (masstheta4 * gravity * (Dtheta1theta2 + Dtheta2theta3 + Dtheta3theta4)) +(masslinktheta3theta4  * gravity *(Dtheta1theta2 + Dtheta2theta3 + Dtheta3theta4/2))+(masstheta3 * gravity * (Dtheta1theta2 + Dtheta2theta3))+(masslinktheta2theta3 * gravity * (Dtheta1theta2 + Dtheta2theta3/2))+(masstheta2 * gravity * Dtheta1theta2)+(masslinktheta1theta2 * gravity * Dtheta1theta2/2)


print ("Torque 1: %.2f" % torque1)
print ("Torque 2: %.2f" %torque2)
print ("Torque 3: %.2f" %torque3)
print ("Torque 4: %.2f" %torque4)

