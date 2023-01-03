# BCI_TIAGo_Control_with_EEG_Using_Eye_Artifacts
Abstract
Human-robot interaction is a rapidly developing field and robots have been taking more
active roles in our daily lives. Patient care is one of the fields in which robots are becoming
more present, especially for disabled people. Neurodegenerative disordered people
cannot consciously or voluntarily produce movements other than those involving the eyes
or eyelids. In this context, BCI systems present an alternative way to communicate or
interact with the external world. In order to improve the lives of disabled people, this
paper presents a novel brain-computer interface to control an assistive robot with user’s
eye artifacts. In this study, eye artifacts that contaminate the EEG signals are considered
a valuable source of information thanks to their high signal-to-noise ratio and intentional
generation. The proposed methodology detects eye artifacts from EEG signals through
characteristic shapes that occur during the events. The lateral movements are distinguished
by their ordered peak and valley formation and the opposite phase of the signals
measured at F7 and F8 channels. This work, as far as the author’s knowledge is the first
method that used this behavior to detect lateral eye movements. For the blinks detection,
a double-thresholding method is proposed by the author to catch both weak blinks as well
as regular ones, differentiating itself from the other algorithms in the literature that normally
uses only one threshold. Real-time detected events with their virtual time stamps
are fed into a second algorithm, to further distinguish between double and quadruple
blinks from single blinks occurrence frequency. After testing the algorithm offline and
in real-time, the algorithm is implemented on the device. The created BCI was able to
control an assistive robot through a graphical user interface. The validation experiment
proves that the developed BCI is able to control the robot.

Keywords: BCI, HRI, EEG, Eye Artifacts, assistive robot

