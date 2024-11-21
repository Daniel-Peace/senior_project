# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This class contains members and functions for holding and working with information about
# integrated AI and ML models. When the pipeline is started, this class is used to store the
# configuration data for each model in "model_configs.json".
# -------------------------------------------------------------------------------------------

from casualty import Casualty

class Model:
    # constructor
    def __init__(
            self, 
            name                            = "model", 
            weight                          = 0, 
            casualty                        = Casualty(), 
            has_predicted                   = False, 
            coherent_dependent              = False, 
            determines_coherency            = False,
            predicts_severe_hemorrhage      = False,
            predicts_respiratory_distress   = False,
            predicts_heart_rate             = False,
            predicts_respiratory_rate       = False,
            predicts_trauma_head            = False,
            predicts_trauma_torso           = False,
            predicts_trauma_lower_ext       = False,
            predicts_trauma_upper_ext       = False,
            predicts_alertness_ocular       = False,
            predicts_alertness_verbal       = False,
            predicts_alertness_motor        = False
            ):
        
        self.name                           = name
        self.weight                         = weight
        self.casualty                       = casualty
        self.has_predicted                  = has_predicted
        self.coherent_dependent             = coherent_dependent
        self.determines_coherency           = determines_coherency
        self.predicts_severe_hemorrhage     = predicts_severe_hemorrhage
        self.predicts_respiratory_distress  = predicts_respiratory_distress
        self.predicts_heart_rate            = predicts_heart_rate
        self.predicts_respiratory_rate      = predicts_respiratory_rate
        self.predicts_trauma_head           = predicts_trauma_head
        self.predicts_trauma_torso          = predicts_trauma_torso
        self.predicts_trauma_lower_ext      = predicts_trauma_lower_ext
        self.predicts_trauma_upper_ext      = predicts_trauma_upper_ext
        self.predicts_alertness_ocular      = predicts_alertness_ocular
        self.predicts_alertness_verbal      = predicts_alertness_verbal
        self.predicts_alertness_motor       = predicts_alertness_motor

    # resets model
    def reset(self):
        self.has_predicted = False
        self.casualty.reset()

    # prints model members
    def print_self(self):
        print("+----------------------+---------------------+")
        print("| %-21s| %-20s|" % ("name", self.name))
        print("+----------------------+---------------------+")
        print("| %-21s| %-20s|" % ("weight", self.weight))
        print("+----------------------+---------------------+")
        print("| %-21s| %-20s|" % ("has predicted", self.has_predicted))
        print("+----------------------+---------------------+")
        print("| %-21s| %-20s|" % ("coherent dependent", self.coherent_dependent))
        print("+----------------------+---------------------+")
        print("| %-21s| %-20s|" % ("determines coherency", self.determines_coherency))
        print("+----------------------+---------------------+")
        print(f"|{'casualty':^44}|")
        print("+--------------------------------------------+")
        self.casualty.print_self()
        print("+--------------------------------------------+\n")