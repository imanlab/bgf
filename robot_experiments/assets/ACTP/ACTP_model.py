# -*- coding: utf-8 -*-
# RUN IN PYTHON 3
import torch
import torch.nn as nn
from itertools import cycle

class ACTP(nn.Module):
    def __init__(self, device, context_frames):
        super(ACTP, self).__init__()
        self.device = device
        self.context_frames = context_frames
        self.fc0 = nn.Linear(24, 48).to(device)
        self.lstm1 = nn.LSTM(48, 200).to(device)  # tactile
        self.lstm2 = nn.LSTM(200 + 48, 200).to(device)  # tactile
        self.fc1 = nn.Linear(200 + 48, 200).to(device)  # tactile + pos
        self.fc2 = nn.Linear(200, 48).to(device)  # tactile + pos
        self.tan_activation = nn.Tanh().to(device)
        self.relu_activation = nn.ReLU().to(device)

    def forward(self, tactiles, actions):
        state = actions[0]
        # state.to(self.device)
        batch_size__ = tactiles.shape[1]
        outputs = []
        hidden1 = (torch.zeros(1, batch_size__, 200, device=torch.device('cpu'), dtype=torch.float32), torch.zeros(1, batch_size__, 200, device=torch.device('cpu'), dtype=torch.float32))
        hidden2 = (torch.zeros(1, batch_size__, 200, device=torch.device('cpu'), dtype=torch.float32), torch.zeros(1, batch_size__, 200, device=torch.device('cpu'), dtype=torch.float32))

        for index, (sample_tactile, sample_action,) in enumerate(zip(cycle(tactiles[:-1]), actions[1:])):
            # 2. Run through lstm:
            if index > self.context_frames-1:
                out4 = out4.view(batch_size__, -1)
                out1, hidden1 = self.lstm1(out4.view(1, batch_size__, -1), hidden1)
                tiled_action_and_state = torch.cat((state, state, sample_action, sample_action), 1)
                out0 = self.relu_activation(self.fc0(tiled_action_and_state))
                action_and_tactile = torch.cat((out1.view(batch_size__, -1), out0), 1)
                out2, hidden2 = self.lstm2(action_and_tactile.view(1, batch_size__, -1), hidden2)
                lstm_and_prev_tactile = torch.cat((out2.view(batch_size__, -1), out4), 1)
                out3 = self.tan_activation(self.fc1(lstm_and_prev_tactile))
                out4 = self.tan_activation(self.fc2(out3))
                outputs.append(out4.view(batch_size__, -1))
            else:
                out1, hidden1 = self.lstm1(sample_tactile.view(1, batch_size__, -1), hidden1)
                tiled_action_and_state = torch.cat((state, state, sample_action, sample_action), 1)
                out0 = self.relu_activation(self.fc0(tiled_action_and_state))
                action_and_tactile = torch.cat((out1.view(batch_size__, -1), out0), 1)
                out2, hidden2 = self.lstm2(action_and_tactile.view(1, batch_size__, -1), hidden2)
                lstm_and_prev_tactile = torch.cat((out2.view(batch_size__, -1), sample_tactile), 1)
                out3 = self.tan_activation(self.fc1(lstm_and_prev_tactile))
                out4 = self.tan_activation(self.fc2(out3))
                last_output = out4

        outputs = [last_output] + outputs
        return torch.stack(outputs)