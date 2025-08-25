import zmq
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "build/proto")) # Add the build/proto directory to sys.path
import telemetry_pb2  # Generated from telemetry.proto

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import numpy as np
import datetime
import csv
# Maximum number of data points to display
MAX_POINTS = 1000

# Configuration for each subplot:
# Each entry is a dictionary with:
#   - 'channels': number of channels (each gets its own curve)
#   - 'title': plot title
#   - 'overlay': tuple (actual_field, commanded_field) if you want two signals on one plot;
#                otherwise, use None.
PLOT_CONFIGS = [
    {
        'field': 'omega_b2i',
        'channels': 3,
        'title': 'Body Rates (rad/s)',
        'overlay': None,
        'ylim': [-np.pi, np.pi]
    },
    {
        'overlay_pair': ('q_b2i', 'q_i2d'),
        'channels': 4,
        'title': 'Body (q_b2i) and Desired (q_i2d) Quat',
        'overlay': None,
        'ylim': [-1.1, 1.1]
    },
    {
        'overlay_pair': ('r_mass', 'r_mass_commanded'),
        'channels': 3,
        'title': 'Mass Position (m)',
        'ylim': [-.08, .08]
    },
    {
        'field': 'rdot_mass',
        'channels': 3,
        'title': 'Mass Velocity (m/s)',
        'overlay': None,
        'ylim': [-.5, .5]
    },
    {
        'overlay_pair': ('u_actual', 'u_com'),
        'channels': 3,
        'title': 'Control Input (Nm)',
        'ylim': [-.2, 0.35] # check this
    },
    {
        'field': 'theta_hat',
        'channels': 3,
        'title': 'Uncertainty Estimate (m)',
        'overlay': None, 
        'ylim': [-0.1, 0.1] # check this 
    },
    {
        'field': 'omega_d2i_D',
        'channels': 3,
        'title': 'Desired Trajectory Rates (rad/s)',
        'overlay': None,
        'ylim': [-1, 1] # check this 
    }
]

# For CSV logging, define the order and expected number of elements for each field.
CSV_FIELDS = [
    ('time', 1),
    ('omega_b2i', 3),
    ('q_b2i', 4),
    ('q_i2d', 4),
    ('r_mass', 3),
    ('rdot_mass', 3),
    ('r_mass_commanded', 3),
    ('u_com', 3),
    ('u_actual', 3),
    ('theta_hat', 3),
    ('omega_d2i_D', 3)
]

# Colors to use for channels
COLORS_3 = ['r', 'g', 'c']
COLORS_4 = ['y', 'r', 'g', 'c'] # w x y z for eigen quaternion

class WholeSecondAxis(pg.AxisItem):
    def tickStrings(self, values, scale, spacing):
        # Each value is in seconds; round to the nearest whole number.
        return [str(int(round(val))) for val in values]
    
class MultiTelemetryPlotter:
    def __init__(self):
        # Set up ZeroMQ subscriber (TCP)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://155.31.239.181:5555")  # Update if needed
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

        # Data buffers: time plus one buffer per field/channel.
        self.data = {'time': []}
        # For fields that are single (non-overlay) store as: data[field] = [[], [], ...]
        # For overlay pairs, we'll store both separately.
        self.fields = ['omega_b2i', 'q_b2i', 'q_i2d', 'rdot_mass', 'theta_hat', 'omega_d2i_D']
        for field in self.fields:
            self.data[field] = []  # We will later split into channels per plot

        # For overlay pairs, e.g. mass position and control input:
        self.overlay_fields = ['r_mass', 'r_mass_commanded', 'u_actual', 'u_com']
        for field in self.overlay_fields:
            self.data[field] = []  # Each message holds a vector

        # Set up CSV logging.
        self.setup_csv_logging()

        # Create a window with a grid layout. We have 6 subplots.
        self.app = pg.mkQApp("Multi-Field Telemetry Plot")
        self.win = pg.GraphicsLayoutWidget(title="Telemetry Data", show=True)
        self.win.resize(1920,1080)

        self.plots = []   # List of plot items in the order of PLOT_CONFIGS
        self.curves = []  # List of lists of curves for each subplot

        # Create subplots arranged in a 3x2 grid
        rows, cols = 3, 2
        grid_index = 0

        for config in PLOT_CONFIGS:
            # Determine position in grid
            row = grid_index // cols
            col = grid_index % cols
            grid_index += 1

            p = self.win.addPlot(row=row, col=col, title=config['title'])
            p.setTitle(config['title'], color='w', size='22pt')
            p.setLabel('bottom', 'Time (sec)', color='w', size='2pt')
            # Show major grid lines for x and y
            p.showGrid(x=True, y=True, alpha=0.75)

                # Customize title with white text and larger font

            # Customize axis labels with white text and font size
            # p.setLabel('bottom', 'Time (s)', )
            # p.setLabel('left', '', color='w', size='12pt')  # Optional: set left label


            # Optionally, adjust tick spacing for minor ticks:
            # p.getAxis('bottom').setTickSpacing(major=10, minor=5)
            
            p.setYRange(config['ylim'][0], config['ylim'][1])
            # p.enableAutoRange(axis='y', enable=False)  # Disable auto-scaling for Y-axis
            # num_ticks = 7 # Number of ticks including min and max
            # ytick_positions = np.linspace(config['ylim'][0], config['ylim'][1], num_ticks)
            # yticks = [(y, f"{y:.1f}") for y in ytick_positions]  # Format labels
            # p.getAxis('left').setTicks([yticks])


            self.plots.append(p)

            curves_for_plot = []
            channels = config['channels']
            if 'overlay_pair' in config:
                # Overlay: two sets of curves on one plot.
                # First field is actual (solid), second is commanded (dashed).
                # Use same colors for both.
                colors = COLORS_3 if channels == 3 else COLORS_4
                for i in range(channels):
                    pen_actual = pg.mkPen(colors[i], width=1, style=QtCore.Qt.SolidLine)
                    curve_actual = p.plot(pen=pen_actual, name=f"{config['overlay_pair'][0]}[{i}]")
                    curves_for_plot.append({'actual': curve_actual})
                for i in range(channels):
                    pen_commanded = pg.mkPen(colors[i], width=1, style=QtCore.Qt.DashLine)
                    # Plot the commanded signal on the same axes
                    curve_commanded = p.plot(pen=pen_commanded, name=f"{config['overlay_pair'][1]}[{i}]")
                    curves_for_plot[i]['commanded'] = curve_commanded
            else:
                # Single field: plot each channel as its own curve.
                colors = COLORS_3 if channels == 3 else COLORS_4
                for i in range(channels):
                    pen = pg.mkPen(colors[i], width=1)
                    curve = p.plot(pen=pen, name=f"{config['field']}[{i}]")
                    curves_for_plot.append(curve)
            self.curves.append(curves_for_plot)

        # Set up a QTimer to update plots periodically
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(17)  # update every 17 ms (60fps )
    def setup_csv_logging(self):
        # Generate a CSV file name based on the start time (month, day, hour, minute)
        now = datetime.datetime.now()
        filename = now.strftime("logs/%m%d__%H_%M") + ".csv"
        self.csv_filename = filename
        # Open the file for writing.
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Create header row.
        header = []
        for field, count in CSV_FIELDS:
            if count == 1:
                header.append(field)
            else:
                for i in range(count):
                    header.append(f"{field}_{i}")
        self.csv_writer.writerow(header)
        self.csv_file.flush()
        print(f"Logging CSV to {self.csv_filename}")

    def log_message(self, msg):
        # Prepare a row for CSV logging.
        row = []
        # Log the message time.
        row.append(float(msg.time))
        # For each field in CSV_FIELDS (skip time since already logged)
        for field, count in CSV_FIELDS[1:]:
            values = list(getattr(msg, field))
            # Pad with zeros if not enough values.
            if len(values) < count:
                values.extend([0] * (count - len(values)))
            # Append exactly count values.
            row.extend(values[:count])
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def update_plots(self):
        # Poll for new messages (non-blocking)
        while True:
            try:
                serialized_msg = self.socket.recv(flags=zmq.NOBLOCK)
                msg = telemetry_pb2.TelemetryMessage()
                msg.ParseFromString(serialized_msg)
                
                # Log the message to CSV.
                self.log_message(msg)
                
                # Append new time stamp (convert uint64 to float, if needed)
                self.data['time'].append(float(msg.time))
                if len(self.data['time']) > MAX_POINTS:
                    self.data['time'] = self.data['time'][-MAX_POINTS:]

                # For single fields
                for field in self.fields:
                    values = list(getattr(msg, field))
                    self.data[field].append(values)
                    if len(self.data[field]) > MAX_POINTS:
                        self.data[field] = self.data[field][-MAX_POINTS:]

                # For overlay fields
                for field in self.overlay_fields:
                    values = list(getattr(msg, field))
                    self.data[field].append(values)
                    if len(self.data[field]) > MAX_POINTS:
                        self.data[field] = self.data[field][-MAX_POINTS:]
            except zmq.Again:
                break

            # Convert the time buffer to a numpy array.
            t = np.array(self.data['time'])

            if t.size == 1:
                self.start_time = t[0]

            if t.size:
                # Convert absolute time (seconds) to elapsed time in milliseconds since the first timestamp.
                t = (t - self.start_time) / 1000.0
        
        # Loop over all plot configurations in order
        for idx, config in enumerate(PLOT_CONFIGS):
            if 'overlay_pair' in config:
                # This is an overlay plot: update both actual and commanded curves.
                actual_field, commanded_field = config['overlay_pair']
                channels = config['channels']
                data_actual = np.array(self.data[actual_field])
                data_commanded = np.array(self.data[commanded_field])
                for ch in range(channels):
                    if data_actual.size:
                        self.curves[idx][ch]['actual'].setData(t, data_actual[:, ch])
                    if data_commanded.size:
                        self.curves[idx][ch]['commanded'].setData(t, data_commanded[:, ch])
            else:
                # Non-overlay plot: update the single set of curves.
                field = config['field']
                channels = config['channels']
                data = np.array(self.data[field])  # shape: (n_points, channels)
                for ch in range(channels):
                    if data.size:;
                        self.curves[idx][ch].setData(t, data[:, ch])


    def run(self):
        self.app.exec()
        self.csv_file.close()


if __name__ == "__main__":
    plotter = MultiTelemetryPlotter()
    plotter.run()
