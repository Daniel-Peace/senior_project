class Casualty_types:
    # critical options
    SEVERE_HEMORRHAGE       = 0
    RESPIRATORY_DISTRESS    = 1

    # vitals options
    HEART_RATE              = 2
    RESPIRATORY_RATE        = 3

    # injury options
    TRAUMA_HEAD             = 4
    TRAUMA_TORSO            = 5
    TRAUMA_LOWER_EXT        = 6
    TRAUMA_UPPER_EXT        = 7
    ALERTNESS_OCULAR        = 8
    ALERTNESS_VERBAL        = 9
    ALERTNESS_MOTOR         = 10

class Casualtys_status:
    # # default constructor
    # def __init__(self) -> None:
    #     self.april_tag              = -1
    #     self.severe_hemorrhage      = -1
    #     self.respiratory_distress   = -1
    #     self.heart_rate             = -1
    #     self.respiratory_rate       = -1
    #     self.trauma_head            = -1
    #     self.trauma_torso           = -1
    #     self.trauma_lower_ext       = -1
    #     self.trauma_upper_ext       = -1
    #     self.alertness_ocular       = -1
    #     self.alertness_verbal       = -1
    #     self.alertness_motor        = -1


    # constructor with all arguments passed in
    def __init__(self, april_tag = -1, severe_hemorrhage = -1, respiratory_distress = -1, heart_rate = -1, respiratory_rate = -1, trauma_head = -1, trauma_torso = -1, trauma_lower_ext = -1, trauma_upper_ext = -1, alertness_ocular = -1, alertness_verbal = -1, alertness_motor = -1) -> None:
        self.april_tag              = april_tag
        self.severe_hemorrhage      = severe_hemorrhage
        self.respiratory_distress   = respiratory_distress
        self.heart_rate             = heart_rate
        self.respiratory_rate       = respiratory_rate
        self.trauma_head            = trauma_head
        self.trauma_torso           = trauma_torso
        self.trauma_lower_ext       = trauma_lower_ext
        self.trauma_lupper_ext      = trauma_upper_ext
        self.alertness_ocular       = alertness_ocular
        self.alertness_verabl       = alertness_verbal
        self.alertness_motor        = alertness_motor
