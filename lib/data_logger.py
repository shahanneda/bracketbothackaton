import matplotlib.pyplot as plt

class DataLogger:
    def __init__(self):
        self.data = {}
    
    def log(self, **kwargs):
        """
        Log any number of variables provided as keyword arguments.
        """
        for key, value in kwargs.items():
            if key not in self.data:
                self.data[key] = []
            self.data[key].append(value)
    
    def plot(self, x_key='time', y_keys=None):
        """
        Plot the logged data.
        :param x_key: The key to use for the x-axis (default is 'time').
        :param y_keys: A list of keys to plot against x_key. If None, all keys except x_key are plotted.
        """
        if y_keys is None:
            y_keys = [key for key in self.data.keys() if key != x_key]
        
        for y_key in y_keys:
            plt.figure()
            plt.plot(self.data[x_key], self.data[y_key], label=y_key)
            plt.xlabel(x_key)
            plt.ylabel(y_key)
            plt.title(f'{y_key} vs {x_key}')
            plt.legend()
            plt.show() 