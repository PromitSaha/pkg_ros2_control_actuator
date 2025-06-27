import numpy as np

class inv_kinematics:
    def __init__(self,r_B, r_P, gamma_B, gamma_P) -> None:
        self.home_pos= np.array([0, 0, .05]) # home position of the platform

        pi = np.pi

        ## Define the Geometry of the platform
        # Psi_B (Polar coordinates)
        psi_B = np.array([ 
            -gamma_B, 
            gamma_B,
            2*pi/3 - gamma_B, 
            2*pi/3 + gamma_B, 
            2*pi/3 + 2*pi/3 - gamma_B, 
            2*pi/3 + 2*pi/3 + gamma_B])

        # psi_P (Polar coordinates)
        # Direction of the points where the rod is attached to the platform.
        psi_P = np.array([ 
            pi/3 + 2*pi/3 + 2*pi/3 + gamma_P,
            pi/3 + -gamma_P, 
            pi/3 + gamma_P,
            pi/3 + 2*pi/3 - gamma_P, 
            pi/3 + 2*pi/3 + gamma_P, 
            pi/3 + 2*pi/3 + 2*pi/3 - gamma_P])

        # Coordinate of the points where servo arms 
        # are attached to the corresponding servo axis.
        beforeTranspose_B = r_B * np.array( [ 
            [ np.cos(psi_B[0]), np.sin(psi_B[0]), 0],
            [ np.cos(psi_B[1]), np.sin(psi_B[1]), 0],
            [ np.cos(psi_B[2]), np.sin(psi_B[2]), 0],
            [ np.cos(psi_B[3]), np.sin(psi_B[3]), 0],
            [ np.cos(psi_B[4]), np.sin(psi_B[4]), 0],
            [ np.cos(psi_B[5]), np.sin(psi_B[5]), 0] ])
        self.B = np.transpose(beforeTranspose_B)
            
        # Coordinates of the points where the rods 
        # are attached to the platform.
        beforeTranspose_P = r_P * np.array([ 
            [ np.cos(psi_P[0]),  np.sin(psi_P[0]), 0],
            [ np.cos(psi_P[1]),  np.sin(psi_P[1]), 0],
            [ np.cos(psi_P[2]),  np.sin(psi_P[2]), 0],
            [ np.cos(psi_P[3]),  np.sin(psi_P[3]), 0],
            [ np.cos(psi_P[4]),  np.sin(psi_P[4]), 0],
            [ np.cos(psi_P[5]),  np.sin(psi_P[5]), 0] ])
        self.P = np.transpose(beforeTranspose_P)

    # Rotation matrices used later
    def rotX(self, theta):
        rotx = np.array([
            [1,     0    ,    0    ],
            [0,  np.cos(theta), -np.sin(theta)],
            [0,  np.sin(theta), np.cos(theta)] ])
        return rotx

    def rotY(self, theta):    
        roty = np.array([
            [np.cos(theta), 0,  np.sin(theta) ],
            [0         , 1,     0       ],
            [-np.sin(theta), 0,  np.cos(theta) ] ])   
        return roty

    def rotZ(self, theta):    
        rotz = np.array([
            [ np.cos(theta),-np.sin(theta), 0 ],
            [ np.sin(theta), np.cos(theta), 0 ],
            [   0        ,     0      , 1 ] ])   
        return rotz
    
    def solve(self, trans, rotation):
        # Get rotation matrix of platform. RotZ* RotY * RotX -> matmul
        # R = np.matmul( np.matmul(rotZ(rotation[2]), rotY(rotation[1])), rotX(rotation[0]) )
        R = np.matmul( np.matmul(self.rotX(rotation[0]), self.rotY(rotation[1])), self.rotZ(rotation[2]) )

        # Get leg length for each leg
        # leg = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(home_pos[:, np.newaxis], 6, axis=1) + np.matmul(np.transpose(R), P) - B 
        l = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(self.home_pos[:, np.newaxis], 6, axis=1) + np.matmul(R, self.P) - self.B 
        lll = np.linalg.norm(l, axis=0)

        # Position of leg in global frame
        L = l + self.B

        return lll