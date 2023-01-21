

class DS4_Mapper:    
    def __init__(self):
        # mapping buttons
        self.box = 0
        self.cross = 1 
        self.circle = 2
        self.triangle = 3
        self.l1 = 4
        self.r1 = 5
        self.l2 = 6 
        self.r2 = 7 
        self.share = 8 
        self.option = 9
        self.l3 = 10
        self.r3 = 11
        self.ps = 12
        self.touch_pad_b = 13
        
        # mapping axes
        self.l_stick_lr = 0
        self.l_stick_ud = 1
        self.r_stick_lr = 2
        self.r_stick_ud = 5
        self.l2_stick   = 3
        self.r2_stick   = 4