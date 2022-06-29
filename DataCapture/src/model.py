import torch 
import torchvision.transforms as T
import torch.nn as nn

class CNN_ODOM_Model(nn.Module):
    def __init__(self, input_dim):
        super(CNN_ODOM_Model, self).__init__()
        #apply image argumention
        
        if self.training == True:
            self.transform = nn.Sequential(
                T.RandomCrop(96),
                T.RandomApply(torch.nn.ModuleList([
                    T.ColorJitter(),
                    #T.RandomRotation((-15,15)),
                    T.ColorJitter(hue=0.3),
                    T.RandomErasing(0.3),
                    T.RandomInvert(0.3),
                    T.RandomGrayscale(0.3),
                    T.GaussianBlur((3,3)),
                ]), p=0.5),
                T.ConvertImageDtype(torch.float),
                T.Normalize(mean = (0.5, 0.5, 0.5), std = (0.5, 0.5, 0.5))
            )
        elif self.training == False:
            self.transform = nn.Sequential(
                T.RandomCrop(96),
                T.ConvertImageDtype(torch.float),
                T.Normalize(mean = (0.5, 0.5, 0.5), std = (0.5, 0.5, 0.5))
            )
        
        
        #image CNN layers
        self.cnn = nn.Sequential(
            
            nn.Conv2d(3, 24, 5, 1, 1), # [64, 96, 96]
            #nn.BatchNorm2d(24),
            nn.ReLU(),
            nn.MaxPool2d(2, 2, 0),      # [64, 48, 48]

            nn.Conv2d(24, 36, 5, 1, 1), # [128, 48, 48]
            #nn.BatchNorm2d(36),
            nn.ReLU(),
            nn.MaxPool2d(2, 2, 0),      # [128, 24, 24]
            
            nn.Conv2d(36, 48, 3, 1, 1), # [128, 48, 48]
            #nn.BatchNorm2d(48),
            nn.ReLU(),
            nn.MaxPool2d(2, 2, 0),      # [128, 12, 12]
            
            nn.Conv2d(48, 96, 3, 1, 1), # [128, 12, 12]
            #nn.BatchNorm2d(96),
            nn.ReLU(),
            nn.MaxPool2d(2, 2, 0),      # [128, 6, 6]
        )
        
        #image softmax layer
        self.fc = nn.Sequential(
        nn.Linear(96*5*5,1000),
        nn.ReLU(),
        nn.Linear(1000, 100),
        nn.ReLU(),
        nn.Linear(100, 50),
        nn.ReLU(),
        nn.Linear(50, 10),
        )
        #odometry fully connected layer
        self.odomlayers = nn.Sequential(
            nn.Linear(3, 10),
            nn.ReLU(),
            nn.Linear(10, 10),
            nn.ReLU(),
            nn.Linear(10, 10),
            nn.ReLU(),
            nn.Linear(10, 10),
        )
        #recurrent layers
        self.rnn=nn.RNN(input_size=10,hidden_size=10,num_layers=1,batch_first=True)
        
        self.lstm = nn.LSTM(input_size=10, hidden_size=10, num_layers=1,batch_first=True)
        
        #the last fully connected layer
        self.lastlayers = nn.Sequential(
            nn.Linear(20, 2), #return 2
        )
    def forward(self, x):
        #reshape 1-D vector to RGB image tensor
        x1 = torch.reshape((x[:,:-3]),(x.size(0),128,128,3))# to 128x128 image
        x1=x1.permute(0,3,1,2)
        #apply image transform
        x1=self.transform(x1)

        '''    
        img = x1[0].cpu()
        npimg = img.numpy().astype('uint8')
        plt.imshow(np.transpose(npimg, (1,2,0)), interpolation='nearest')
        '''      
        
        #image layers
        x1=self.cnn(x1)
        #x1 = x1.view(x1.size()[0], -1)
        x1=torch.flatten(x1, start_dim=1)
        x1=self.fc(x1)
        
        

        #odometry layers
        #x2 = torch.reshape((x[:,49153]),(x.size(0),1))
        
        x2 = x[:,-3:]
        x2 = self.odomlayers(x2)
        x=torch.cat((x1,x2),1)
        
        #x=x1
        '''
        h0=torch.zeros(1,x.size(0),x.size(1)).to(device)
        c0=torch.zeros(1,x.size(0),x.size(1)).to(device)
        x = x.view(x.size(0), x.size(1), -1)
        x = x.permute(0, 2, 1)
        x,_=self.lstm(x,(h0,c0))
        #x,_=self.rnn(x,h0)
        #x=x.reshape(x.shape[0],-1)  #for rnn 
        x=x[:,-1,:] # for lstms
        '''
        
        #last fully connected layer
        x = self.lastlayers(x)
        


        return x