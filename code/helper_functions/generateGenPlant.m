function [GenPlant, Win, Wout] = generateGenPlant(Plant, Wu, We, W1, W2)
    Wout = blkdiag(Wu, We, eye(size(Plant, 1)));
    Win = blkdiag(W1, W2, eye(size(Plant, 2)));
    GenPlant = Wout * [zeros(size(Plant)) zeros(size(Plant)) eye(size(Plant));
                       eye(size(Plant))  -Plant              -Plant;
                       eye(size(Plant))  -Plant              -Plant;] * Win;

    GenPlant.InputName = {'RootMycD Reference', 'RootMycQ Reference', ...
                          'BldPitchD Disturbance', 'BldPitchQ Disturbance', ...
                          'BldPitchD Control Input', 'BldPitchQ Control Input'};
    GenPlant.OutputName = {'Weighted BldPitchD Control Input', 'Weighted BldPitchQ Control Input', ...
            'Weighted RootMycD Tracking Error', 'Weighted RootMycQ Tracking Error', ...
            'Measured RootMycD Tracking Error', 'Measured RootMycQ Tracking Error'};
end