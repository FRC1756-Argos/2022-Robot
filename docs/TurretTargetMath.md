
### Turret Target Math

This math gives a target angle based upon the distance to the hub

$$T = \theta - \psi$$
$$\psi = \sin^{-1}\left(\frac{d_1 \sin {\alpha}}{d_2}\right)$$
$$d_2 = \sqrt{{d_1}^2 + 81 - 18d_1 \cos{\alpha}}$$
$$\alpha = 180° - \beta$$
Where:
$T$ = Target Angle
$\theta$ = Current Turret Angle
$\beta$ = Absolute value of yaw of target
$d_1$ = Distance to target from camera
$d_2$ = Distance to target from turret center of rotation
$\alpha$ = The opposite angle of $d_2$

(see "HowIsThisATriangle.png")
