from ActionSet import ActionSet
from slider_gui.actions.Pr2SpeakAction import Pr2SpeakAction

class DefaultSpeechAction(ActionSet):

    def __init__(self):
        super(DefaultSpeechAction, self).__init__()
        speech = Pr2SpeakAction()
        self.add_action(speech)
