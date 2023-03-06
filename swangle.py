import numpy as np
import matplotlib.pyplot as plt
import tflearn
from math import tanh

# https://docs.google.com/spreadsheets/d/1zygoaG0iNYMyMnAb6GsIandL4cDL_8px4BqapFtllPI/edit#gid=1889405692
adcValues = np.array([[1949, 1937.5, 1929, 1914, 1890, 1852, 1837, 1803, 1779, 1747, 1720, 1686, 1614, 1655, 1587, 1551, 1506, 1480, 1442, 1401, 1366, 1314, 1282, 1241, 1200, 1132, 1068, 997, 923, 853, 780, 716, 646, 582, 514, 1977, 1993, 2012, 2033, 2057.5, 2070, 2084, 2105.5, 2126, 2145, 2161, 2186, 2202.5, 2224, 2248, 2303, 2275, 2328, 2361, 2393, 2422, 2448, 2475, 2505.5, 2547, 2583, 2611, 2655, 2714, 2779, 2841, 2912, 2983.5, 3055, 3146, 3215, 3281, 3358]])
swangleValues = np.array([0, 1, 2, 4, 6, 10, 12, 15, 18, 21, 24, 27, 33, 30, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63, 66, 70.1, 75, 80, 85, 90, 95, 100, 105, 110, 118, -2, -4, -6, -8, -10, -12, -14, -16, -18, -20, -22, -24, -26, -28, -30.2, -36, -33, -39, -42, -45, -48, -51, -54, -57, -60, -63.2, -66, -70, -75, -80, -85, -90, -95, -100, -105.2, -110, -115, -120.5])

scaledAdc = (adcValues - np.min(adcValues)) / (np.max(adcValues) - np.min(adcValues))
class EarlyStoppingCallback(tflearn.callbacks.Callback):
    def __init__(self, val_acc_thresh):
        """ Note: We are free to define our init function however we please. """
        self.val_acc_thresh = val_acc_thresh

    def on_epoch_end(self, training_state):
        """ """
        # Apparently this can happen.
        if training_state.val_acc is None: return
        if training_state.val_acc < self.val_acc_thresh:
            raise StopIteration
# Initializae our callback.
early_stopping_cb = EarlyStoppingCallback(val_acc_thresh=0.95)

input_ = tflearn.input_data(shape=[None,1])
layer1 = tflearn.fully_connected(input_, 3, activation='tanh')
layer2 = tflearn.fully_connected(layer1, 1, activation='linear')
regression = tflearn.regression(layer2, optimizer='sgd', loss='mean_square',
                                metric='R2', learning_rate=0.001)
m = tflearn.DNN(regression)
# Comment the following lines in to train the model
# m.fit(scaledAdc.T.tolist(), np.array([swangleValues]).T.tolist(), n_epoch=250000,
#         show_metric=True, snapshot_epoch=False, callbacks=early_stopping_cb)

# print("L1 W", m.get_weights(layer1.W))
# print("L1 b", m.get_weights(layer1.b))
# print("L2 W", m.get_weights(layer2.W))
# print("L2 b", m.get_weights(layer2.b))

# x_test = np.array([np.linspace(-0.05, 1.05, 500)]).T
# y_test = m.predict(x_test.tolist())

# Trained model
W1 = [4.423423, 4.352119, 3.5617096]
b1 = [-2.1946218, -0.34729433, -2.9068983]
W2 = [-49.75351, -42.093838, -50.62649]
b2 = 1.5765486

x_test2 = np.array([np.linspace(-0.05, 1.05, 100)]).T
z1_0 = W1[0] * x_test2 + b1[0]
z1_1 = W1[1] * x_test2 + b1[1]
z1_2 = W1[2] * x_test2 + b1[2]
a1_0 = np.array([tanh(float(z)) for z in z1_0])
a1_1 = np.array([tanh(float(z)) for z in z1_1])
a1_2 = np.array([tanh(float(z)) for z in z1_2])
z2_0 = W2[0] * a1_0 + W2[1] * a1_1 + W2[2] * a1_2 + b2

plt.figure()
# plt.plot(x_test * (np.max(adcValues) - np.min(adcValues)) +  np.min(adcValues), y_test, 'g')
plt.plot(x_test2 * (np.max(adcValues) - np.min(adcValues)) +  np.min(adcValues), z2_0, 'r')
plt.plot(adcValues[0], swangleValues, 'b.')
plt.xlabel("ADC Value")
plt.ylabel("Steering Wheel Angle")
plt.legend(["ML Fit", "ML Test", "Data Points"])
plt.show()