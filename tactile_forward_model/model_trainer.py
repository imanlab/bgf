# -*- coding: utf-8 -*-
# RUN IN PYTHON 3
import os
import csv
import copy
import numpy as np

from tqdm import tqdm
from datetime import datetime
from torch.utils.data import Dataset

import torch
import torch.nn as nn
import torch.optim as optim
import torchvision

from ACTP.ACTP_model import ACTP

seed = 42

torch.manual_seed(seed)
torch.backends.cudnn.benchmark = False
torch.backends.cudnn.deterministic = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # use gpu if available


class BatchGenerator:
    def __init__(self, train_percentage, train_data_dir, batch_size, image_size):
        self.batch_size = batch_size
        self.image_size = image_size
        self.train_data_dir = train_data_dir
        self.train_percentage = train_percentage
        self.data_map = []
        with open(train_data_dir + 'map.csv', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                self.data_map.append(row)

    def load_full_data(self):
        dataset_train = FullDataSet(self.data_map, self.train_percentage, self.train_data_dir, self.image_size, train=True)
        dataset_validate = FullDataSet(self.data_map, self.train_percentage, self.train_data_dir, self.image_size, validation=True)
        transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
        train_loader = torch.utils.data.DataLoader(dataset_train, batch_size=self.batch_size, shuffle=True, num_workers=4, pin_memory=True, drop_last=True)
        validation_loader = torch.utils.data.DataLoader(dataset_validate, batch_size=self.batch_size, shuffle=True, num_workers=4, pin_memory=True, drop_last=True)
        self.data_map = []
        return train_loader, validation_loader


class FullDataSet:
    def __init__(self, data_map, train_percentage, train_data_dir, image_size, train=False, validation=False):
        self.train_data_dir = train_data_dir
        self.image_size = image_size
        if train:
            self.samples = data_map[1:int((len(data_map) * train_percentage))]
        if validation:
            self.samples = data_map[int((len(data_map) * train_percentage)): -1]
        data_map = None

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        value = self.samples[idx]
        robot_data = np.load(self.train_data_dir + value[0])

        if self.image_size == 0:
            tactile_data = np.load(self.train_data_dir + value[1])
            experiment_number = np.load(self.train_data_dir + value[2])
            time_steps = np.load(self.train_data_dir + value[3])
        else:
            tactile_data = []
            for image_name in np.load(self.train_data_dir + value[2]):
                tactile_data.append(np.load(self.train_data_dir + image_name))
            tactile_data = np.array(tactile_data)
            experiment_number = np.load(self.train_data_dir + value[3])
            time_steps = np.load(self.train_data_dir + value[4])

        return [robot_data.astype(np.float32), tactile_data.astype(np.float32), experiment_number, time_steps]


class UniversalModelTrainer:
    def __init__(self, model, criterion, image_size, model_save_path, model_name, epochs, batch_size,
                 learning_rate, context_frames, sequence_length, train_percentage, validation_percentage, train_data_dir):
        self.epochs = epochs
        self.batch_size = batch_size
        self.learning_rate = learning_rate
        self.context_frames = context_frames
        self.sequence_length = sequence_length
        self.train_percentage = train_percentage
        self.validation_percentage = validation_percentage
        self.model_name = model_name
        self.model_save_path = model_save_path
        self.model = model
        self.image_size = image_size
        self.train_data_dir = train_data_dir

        BG = BatchGenerator(self.train_percentage, self.train_data_dir, self.batch_size, self.image_size)
        self.train_full_loader, self.valid_full_loader = BG.load_full_data()

        if criterion == "L1":
            self.criterion = nn.L1Loss()
        if criterion == "L2":
            self.criterion = nn.MSELoss()
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)

    def train_full_model(self):
        best_training_loss = 100.0
        training_val_losses = []
        progress_bar = tqdm(range(0, self.epochs))
        for epoch in progress_bar:
            model_save = ""
            self.train_loss = 0.0
            self.val_loss = 0.0

            # trainging
            for index, batch_features in enumerate(self.train_full_loader):
                self.optimizer.zero_grad()
                action = batch_features[0].squeeze(-1).permute(1, 0, 2).to(device)
                if self.image_size > 0:
                    tactile = batch_features[1].permute(1, 0, 4, 3, 2).to(device)
                else:
                    tactile = torch.flatten(batch_features[1], start_dim=2).permute(1, 0, 2).to(device)
                loss = self.run_batch(tactile, action, train=True)
                # progress_bar.set_description("epoch: {}, ".format(epoch) + "loss: {:.4f}, ".format(float(loss)) + "mean loss: {:.4f}, ".format(self.train_loss/(index+1)))
                train_max_index = index

            # validation
            for index, batch_features in enumerate(self.valid_full_loader):
                self.optimizer.zero_grad()
                action = batch_features[0].squeeze(-1).permute(1, 0, 2).to(device)
                if self.image_size > 0:
                    tactile = batch_features[1].permute(1, 0, 4, 3, 2).to(device)
                else:
                    tactile = torch.flatten(batch_features[1], start_dim=2).permute(1, 0, 2).to(device)
                loss = self.run_batch(tactile, action, validation=True)
                # progress_bar.set_description("epoch: {}, ".format(epoch) + "VAL loss: {:.4f}, ".format(float(loss)) + "VAL mean loss: {:.4f}, ".format(self.val_loss / (index+1)))
                val_max_index = index

            training_val_losses.append([self.train_loss/(train_max_index+1), self.val_loss/(val_max_index+1)])
            np.save(self.model_save_path + "train_val_losses", np.array(training_val_losses))

            # early stopping and saving:
            if best_training_loss > self.val_loss/(val_max_index+1):
                best_training_loss = self.val_loss/(val_max_index+1)
                torch.save(self.model, self.model_save_path + self.model_name)
                model_save = "saved model"

            print("Training mean loss: {:.4f} || Validation mean loss: {:.4f} || {}".format(self.train_loss/(train_max_index+1), self.val_loss/(val_max_index+1), model_save))

    def run_batch(self, tactile, action, train=False, validation=False):
        tactile_predictions = self.model.forward(tactiles=tactile, actions=action)  # Step 3. Run our forward pass.
        loss = self.criterion(tactile_predictions, tactile[self.context_frames:])

        if train:
            loss.backward()
            self.optimizer.step()
            self.train_loss += loss.item()
        elif validation:
            self.val_loss += loss.item()

        return loss.item()

def main():
    model_save_path = "/home/kia/Kiyanoush/UoLincoln/Projects/Tactile_control/tactile_prediction/uninversal_trainer/ACTP/saved_models/"
    train_data_dir = "/home/kia/Kiyanoush/UoLincoln/Projects/Tactile_control/data_set/train_image_dataset_kins/"

    # unique save title:
    model_save_path = model_save_path + "model_" + datetime.now().strftime("%d_%m_%Y_%H_%M/")
    os.mkdir(model_save_path)

    epochs = 60
    batch_size = 128
    learning_rate = 1e-3
    context_frames = 10
    sequence_length = 20
    train_percentage = 0.9
    validation_percentage = 0.1
    image_size = 0  # set to zero if linear data
    criterion = "L1"
    model_name = "ACTP"
    model = ACTP(device=device, context_frames=context_frames)

    UMT = UniversalModelTrainer(model, criterion, image_size, model_save_path, model_name,
                          epochs, batch_size, learning_rate, context_frames, sequence_length,
                          train_percentage, validation_percentage, train_data_dir)
    UMT.train_full_model()

if __name__ == "__main__":
    main()
