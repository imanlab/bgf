# -*- coding: utf-8 -*-
# RUN IN PYTHON 3
import os
import csv
import cv2
import numpy as np

from pickle import load
from torch.utils.data import Dataset

import torch
import torch.nn as nn
import torchvision

from ACTP.ACTP_model import ACTP
# from ACTVP.ACTVP_model import ACTVP

# from PMN.PixelMotionNet import PixelMotionNet
# from PMN.PixelMotionNet import ConvLSTMCell

# from PMN_AC.AC_PMN import ACPixelMotionNet
# from PMN_AC_NA.AC_PMN import ACPixelMotionNet

# from MLP.simple_MLP_model import simple_MLP
# from MLP_AC.simple_ACMLP_model import simple_ACMLP


seed = 42

torch.manual_seed(seed)
torch.backends.cudnn.benchmark = False
torch.backends.cudnn.deterministic = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # use gpu if available


class BatchGenerator:
    def __init__(self, test_data_dir, batch_size, image_size, trial_number):
        self.batch_size = batch_size
        self.image_size = image_size
        self.trial_number = trial_number
        self.test_data_dir = test_data_dir + 'test_trial_' + str(self.trial_number) + '/'
        self.data_map = []
        print(self.test_data_dir + 'map_' + str(self.trial_number) + '.csv')
        with open(self.test_data_dir + 'map_' + str(self.trial_number) + '.csv', 'r') as f:  # rb
            reader = csv.reader(f)
            for row in reader:
                self.data_map.append(row)

    def load_full_data(self):
        dataset_test = FullDataSet(self.data_map, self.test_data_dir, self.image_size)
        transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
        test_loader = torch.utils.data.DataLoader(dataset_test, batch_size=self.batch_size, shuffle=False, num_workers=4, drop_last=True)
        self.data_map = []
        return test_loader


class FullDataSet:
    def __init__(self, data_map, test_data_dir, image_size):
        self.image_size = image_size
        self.test_data_dir = test_data_dir
        self.samples = data_map[1:]
        data_map = None

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        value = self.samples[idx]
        robot_data = np.load(self.test_data_dir + value[0])

        if self.image_size == 0:
            tactile_data = np.load(self.test_data_dir + value[1])
            experiment_number = np.load(self.test_data_dir + value[2])
            time_steps = np.load(self.test_data_dir + value[3])
            slip_label = np.load(self.test_data_dir + value[4])
            failure_label = np.load(self.test_data_dir + value[5])
        else:
            tactile_data = []
            for image_name in np.load(self.test_data_dir + value[2]):
                tactile_data.append(np.load(self.test_data_dir + image_name))
            tactile_data = np.array(tactile_data)
            experiment_number = np.load(self.test_data_dir + value[3])
            time_steps = np.load(self.test_data_dir + value[4])

        return [robot_data.astype(np.float32), tactile_data.astype(np.float32), experiment_number, time_steps, slip_label, failure_label]


class UniversalModelTester:
    def __init__(self, model, number_of_trials, test_data_dir, image_size, criterion, model_path, data_save_path,
                 scaler_dir, model_name, batch_size, context_frames, sequence_length, SV2P_latent_channels=False):
        self.number_of_trials = number_of_trials
        self.batch_size = batch_size
        self.context_frames = context_frames
        self.sequence_length = sequence_length
        self.model_name = model_name
        self.model_save_path = model_path
        self.data_save_path = data_save_path
        self.test_data_dir = test_data_dir
        self.scaler_dir = scaler_dir
        self.model = model
        self.image_size = image_size
        self.SV2P_latent_channels = SV2P_latent_channels
        if criterion == "L1":
            self.criterion = nn.L1Loss()
        if criterion == "L2":
            self.criterion = nn.MSELoss()
        self.load_scalars()

    def test_model(self):
        for trial in self.number_of_trials:
            print(trial)
            BG = BatchGenerator(self.test_data_dir, self.batch_size, self.image_size, trial)
            self.test_full_loader = BG.load_full_data()

            for index, batch_features in enumerate(self.test_full_loader):
                tactile_predictions, tactile_groundtruth, loss = self.run_batch(batch_features[1], batch_features[0])
                if index == 0:
                    if self.image_size == 0:
                        prediction_data = np.array(tactile_predictions.permute(1, 0, 2).cpu().detach())
                        groundtruth_data = np.array(tactile_groundtruth.permute(1, 0, 2).cpu().detach())
                        slip_data = np.array(batch_features[-2])
                        failure_data = np.array(batch_features[-1])
                    else:
                        prediction_data = np.array(tactile_predictions.permute(1, 0, 2, 3, 4).cpu().detach())
                        groundtruth_data = np.array(tactile_groundtruth.permute(1, 0, 2, 3, 4).cpu().detach())
                else:
                    if self.image_size == 0:
                        prediction_data = np.concatenate((prediction_data, np.array(tactile_predictions.permute(1, 0, 2).cpu().detach())), axis=0)
                        groundtruth_data = np.concatenate((groundtruth_data, np.array(tactile_groundtruth.permute(1, 0, 2).cpu().detach())), axis=0)
                        slip_data = np.concatenate((slip_data, np.array(batch_features[-2])))
                        failure_data = np.concatenate((failure_data, np.array(batch_features[-1])))
                    else:
                        prediction_data = np.concatenate((prediction_data, np.array(tactile_predictions.permute(1, 0, 2, 3, 4).cpu().detach())), axis=0)
                        groundtruth_data = np.concatenate((groundtruth_data, np.array(tactile_groundtruth.permute(1, 0, 2, 3, 4).cpu().detach())), axis=0)

            # prediction_data_descaled, groundtruth_data_descaled = self.scale_back(np.array(prediction_data), np.array(groundtruth_data))
            self.save_trial(prediction_data, groundtruth_data, slip_data, failure_data, trial_number=trial)

    def scale_back(self, tactile_data, groundtruth_data):
        pt_descalled_data = []
        gt_descalled_data = []
        if self.image_size == 0:
            (ptx, pty, ptz) = np.split(tactile_data, 3, axis=2)
            (gtx, gty, gtz) = np.split(groundtruth_data, 3, axis=2)
            for time_step in range(tactile_data.shape[0]):
                xela_ptx_inverse_minmax = self.min_max_scalerx_full_data.inverse_transform(ptx[time_step])
                xela_pty_inverse_minmax = self.min_max_scalery_full_data.inverse_transform(pty[time_step])
                xela_ptz_inverse_minmax = self.min_max_scalerz_full_data.inverse_transform(ptz[time_step])
                xela_ptx_inverse_full = self.scaler_tx.inverse_transform(xela_ptx_inverse_minmax)
                xela_pty_inverse_full = self.scaler_ty.inverse_transform(xela_pty_inverse_minmax)
                xela_ptz_inverse_full = self.scaler_tz.inverse_transform(xela_ptz_inverse_minmax)
                pt_descalled_data.append(np.concatenate((xela_ptx_inverse_full, xela_pty_inverse_full, xela_ptz_inverse_full), axis=1))

                xela_gtx_inverse_minmax = self.min_max_scalerx_full_data.inverse_transform(gtx[time_step])
                xela_gty_inverse_minmax = self.min_max_scalery_full_data.inverse_transform(gty[time_step])
                xela_gtz_inverse_minmax = self.min_max_scalerz_full_data.inverse_transform(gtz[time_step])
                xela_gtx_inverse_full = self.scaler_tx.inverse_transform(xela_gtx_inverse_minmax)
                xela_gty_inverse_full = self.scaler_ty.inverse_transform(xela_gty_inverse_minmax)
                xela_gtz_inverse_full = self.scaler_tz.inverse_transform(xela_gtz_inverse_minmax)
                gt_descalled_data.append(np.concatenate((xela_gtx_inverse_full, xela_gty_inverse_full, xela_gtz_inverse_full), axis=1))
        else:
            for time_step in range(tactile_data.shape[0]):
                # convert the image back to the 48 taxel features:
                sequence_p = []
                sequence_g = []
                for ps in range(tactile_data.shape[1]):
                    sequence_p.append(cv2.resize(torch.tensor(tactile_data)[time_step][ps].permute(1, 2, 0).numpy(), dsize=(4, 4), interpolation=cv2.INTER_CUBIC).flatten())
                    sequence_g.append(cv2.resize(torch.tensor(groundtruth_data)[time_step][ps].permute(1, 2, 0).numpy(), dsize=(4, 4), interpolation=cv2.INTER_CUBIC).flatten())

                (ptx, pty, ptz) = np.split(np.array(sequence_p), 3, axis=1)
                (gtx, gty, gtz) = np.split(np.array(sequence_g), 3, axis=1)

                xela_ptx_inverse_minmax = self.min_max_scalerx_full_data.inverse_transform(ptx)
                xela_pty_inverse_minmax = self.min_max_scalery_full_data.inverse_transform(pty)
                xela_ptz_inverse_minmax = self.min_max_scalerz_full_data.inverse_transform(ptz)
                xela_ptx_inverse_full = self.scaler_tx.inverse_transform(xela_ptx_inverse_minmax)
                xela_pty_inverse_full = self.scaler_ty.inverse_transform(xela_pty_inverse_minmax)
                xela_ptz_inverse_full = self.scaler_tz.inverse_transform(xela_ptz_inverse_minmax)
                pt_descalled_data.append(np.concatenate((xela_ptx_inverse_full, xela_pty_inverse_full, xela_ptz_inverse_full), axis=1))

                xela_gtx_inverse_minmax = self.min_max_scalerx_full_data.inverse_transform(gtx)
                xela_gty_inverse_minmax = self.min_max_scalery_full_data.inverse_transform(gty)
                xela_gtz_inverse_minmax = self.min_max_scalerz_full_data.inverse_transform(gtz)
                xela_gtx_inverse_full = self.scaler_tx.inverse_transform(xela_gtx_inverse_minmax)
                xela_gty_inverse_full = self.scaler_ty.inverse_transform(xela_gty_inverse_minmax)
                xela_gtz_inverse_full = self.scaler_tz.inverse_transform(xela_gtz_inverse_minmax)
                gt_descalled_data.append(np.concatenate((xela_gtx_inverse_full, xela_gty_inverse_full, xela_gtz_inverse_full), axis=1))

        return np.array(pt_descalled_data), np.array(gt_descalled_data)

    def run_batch(self, tactile, action):
        if self.image_size == 0:
            action = action.squeeze(-1).permute(1, 0, 2).to(device)
            tactile = torch.flatten(tactile, start_dim=2).permute(1, 0, 2).to(device)
        else:
            tactile = tactile.permute(1, 0, 4, 3, 2).to(device)
            action = action.squeeze(-1).permute(1, 0, 2).to(device)

        if self.model_name == "CDNA":
            tactile_predictions = self.CDNA_pass_through(tactiles=tactile, actions=action)
        elif self.model_name == "SV2P":
            tactile_predictions = self.SV2P_pass_through(tactiles=tactile, actions=action)
        else:
            tactile_predictions = self.model.forward(tactiles=tactile, actions=action)  # Step 3. Run our forward pass.


        if self.model_name == "MLP" or self.model_name == "MLP-AC":

            tactile_predictions = torch.tensor(np.array(np.split(np.array(tactile_predictions.cpu().detach()), self.sequence_length, axis=1))).to(device)

        loss = self.criterion(tactile_predictions, tactile[self.context_frames:])
        return tactile_predictions, tactile[self.context_frames:], loss.item()

    def CDNA_pass_through(self, tactiles, actions):
        hidden = None
        outputs = []
        state = actions[0].to(device)
        with torch.no_grad():
            for index, (sample_tactile, sample_action) in enumerate(zip(tactiles[0:-1].squeeze(), actions[1:].squeeze())):
                state_action = torch.cat((state, sample_action), 1)
                tsa = torch.cat(8*[torch.cat(8*[state_action.unsqueeze(2)], axis=2).unsqueeze(3)], axis=3)
                if index > self.context_frames-1:
                    predictions_t, hidden, cdna_kerns_t, masks_t = self.model(predictions_t, conditions=tsa, hidden_states=hidden)
                    outputs.append(predictions_t)
                else:
                    predictions_t, hidden, cdna_kerns_t, masks_t = self.model(sample_tactile, conditions=tsa, hidden_states=hidden)
                    last_output = predictions_t
        outputs = [last_output] + outputs

        return torch.stack(outputs)

    def SV2P_pass_through(self, tactiles, actions):
        hidden = None
        outputs = []
        state = actions[0].to(device)

        samples = torch.normal(0.0, 1.0, size=(self.batch_size, self.SV2P_latent_channels, 8, 8)).to(device)
        prior_sample = samples
        pMean, pSTD = None, None

        with torch.no_grad():
            for index, (sample_tactile, sample_action) in enumerate(zip(tactiles[0:-1].squeeze(), actions[1:].squeeze())):
                state_action = torch.cat((state, sample_action), 1)
                tsa = torch.cat(8*[torch.cat(8*[state_action.unsqueeze(2)], axis=2).unsqueeze(3)], axis=3)
                tsap = torch.cat([prior_sample, tsa], axis=1)
                if index > self.context_frames-1:
                    predictions_t, hidden, cdna_kerns_t, masks_t = self.model(predictions_t, conditions=tsap, hidden_states=hidden)
                    outputs.append(predictions_t)
                else:
                    predictions_t, hidden, cdna_kerns_t, masks_t = self.model(sample_tactile, conditions=tsap, hidden_states=hidden)
                    last_output = predictions_t

        outputs = [last_output] + outputs

        return torch.stack(outputs)

    def save_trial(self, data_prediction, data_gt, slip_data, failure_data, trial_number):
        # meta_data = np.load(self.test_data_dir + 'test_trial_' + str(trial_number) + '/trial_meta_0.npy', allow_pickle=True)
        path_save = self.data_save_path + "test_trial_" + str(trial_number) + '/'
        try:
            os.mkdir(path_save)
        except:
            pass
        np.save(path_save + "prediction_data", data_prediction)
        np.save(path_save + "groundtruth_data", data_gt)
        # np.save(path_save + "meta_data", meta_data)
        np.save(path_save + "slip_data", slip_data)
        np.save(path_save + "failure_data", failure_data)

    def load_scalars(self):
        self.scaler_tx = load(open(self.scaler_dir + "tactile_standard_scaler_x.pkl", 'rb'))
        self.scaler_ty = load(open(self.scaler_dir + "tactile_standard_scaler_y.pkl", 'rb'))
        self.scaler_tz = load(open(self.scaler_dir + "tactile_standard_scaler_z.pkl", 'rb'))
        self.min_max_scalerx_full_data = load(open(self.scaler_dir + "tactile_min_max_scalar_x.pkl", 'rb'))
        self.min_max_scalery_full_data = load(open(self.scaler_dir + "tactile_min_max_scalar_y.pkl", 'rb'))
        self.min_max_scalerz_full_data = load(open(self.scaler_dir + "tactile_min_max_scalar_z.pkl", 'rb'))


def main():
    model_path = "/home/kia/Kiyanoush/UoLincoln/Projects/Tactile_control/tactile_prediction/uninversal_trainer/ACTP/saved_models/model_29_07_2022_11_27_kins/ACTP"
    data_save_path = "/home/kia/Kiyanoush/UoLincoln/Projects/Tactile_control/tactile_prediction/uninversal_trainer/ACTP/saved_models/model_29_07_2022_11_27_kins/test_results/"
    test_data_dir = "/home/kia/Kiyanoush/UoLincoln/Projects/Tactile_control/data_set/test_image_dataset_kins/"
    scaler_dir = "/home/kia/Kiyanoush/UoLincoln/Projects/Tactile_control/data_set/scalars_kins/"

    number_of_trials = np.arange(849)  # 0-22
    batch_size = 128
    context_frames = 10
    sequence_length = 20
    image_size = 0
    criterion = "L1"
    SV2P_latent_channels = 10

    model = torch.load(model_path).to(device)
    model.eval()
    model_name = "ACTP"

    UMT = UniversalModelTester(model, number_of_trials, test_data_dir, image_size, criterion, model_path,
                         data_save_path, scaler_dir, model_name, batch_size,
                         context_frames, sequence_length, SV2P_latent_channels)  # if not an image set image size to 0
    UMT.test_model()


if __name__ == "__main__":
    main()
