import matplotlib
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import networkx as nx
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import os

# Force interactive backend for container/X11 environments
# matplotlib.use('TkAgg')

# Global configuration
COLORS = ['#3070B3', '#B55CA5', '#F7B11E', '#9FBA36']  # Primary color palette
REFERENCE_COLOR = '#888888'  # Gray color for reference/target signals
LINE_WIDTH = 2.0  # Default line width in points
GRID_COLOR = '#CCCCCC'  # Light gray for grid
AXES_COLOR = '#333333'  # Dark gray for axes
FONT_SIZE_TITLE = 11  # Title font size
FONT_SIZE_LABEL = 11  # Axis label font size
FONT_SIZE_TICK = 11    # Tick label font size
FONT_SIZE_LEGEND = 11  # Legend font size

# Configure matplotlib globally
import matplotlib.font_manager as fm

# Try to load custom Helvetica font
try:
    custom_font_path = os.path.expanduser('~/.local/share/fonts/TMNeueHelvetica-Regular.ttf')
    if os.path.exists(custom_font_path):
        fm.fontManager.addfont(custom_font_path)
        custom_font_name = fm.FontProperties(fname=custom_font_path).get_name()
        plt.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.sans-serif'] = [custom_font_name, 'Helvetica', 'Arial', 'DejaVu Sans']
    else:
        # Fallback to standard Helvetica-style sans-serif fonts
        plt.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.sans-serif'] = ['Helvetica', 'Arial', 'DejaVu Sans']
except:
    # If font loading fails, use standard sans-serif fonts
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.sans-serif'] = ['Helvetica', 'Arial', 'DejaVu Sans']

plt.rcParams['text.usetex'] = False
plt.rcParams['mathtext.fontset'] = 'dejavusans'  # Use sans-serif for math
plt.rcParams['axes.linewidth'] = 1.0
plt.rcParams['axes.edgecolor'] = AXES_COLOR
plt.rcParams['axes.labelcolor'] = AXES_COLOR
plt.rcParams['xtick.color'] = AXES_COLOR
plt.rcParams['ytick.color'] = AXES_COLOR
plt.rcParams['text.color'] = AXES_COLOR
plt.rcParams['axes.spines.top'] = False
plt.rcParams['axes.spines.right'] = False
plt.rcParams['grid.color'] = GRID_COLOR
plt.rcParams['grid.alpha'] = 0.5

# Global figure management
_figure = None
_plots = []  # Store plot data and configs


def _init_figure():
    """Initialize the global figure if not already created."""
    global _figure
    if _figure is None:
        _figure = plt.figure(figsize=(16, 9))
    return _figure


def _get_grid_layout(n):
    """Determine optimal grid layout for n plots on 16:9 screen."""
    if n == 1:
        return 1, 1
    elif n == 2:
        return 1, 2
    elif n == 3:
        return 1, 3
    elif n == 4:
        return 2, 2
    elif n <= 6:
        return 2, 3
    elif n <= 9:
        return 3, 3
    elif n <= 12:
        return 3, 4
    elif n <= 16:
        return 4, 4
    else:
        cols = 4
        rows = (n + cols - 1) // cols
        return rows, cols


def _redraw_all():
    """Redraw all stored plots in the optimal layout."""
    global _figure, _plots
    
    if not _plots:
        return
    
    # Clear figure
    _figure.clear()
    
    # Get layout
    n = len(_plots)
    rows, cols = _get_grid_layout(n)
    
    # Redraw each plot
    for idx, plot_info in enumerate(_plots):
        plot_type = plot_info['type']

        if plot_type == 'timeseries':
            _draw_timeseries(rows, cols, idx + 1, plot_info)
        elif plot_type == 'lineplot':
            _draw_lineplot(rows, cols, idx + 1, plot_info)
        elif plot_type == 'stacked':
            _draw_stacked(rows, cols, idx + 1, plot_info)
        elif plot_type == 'matrix':
            _draw_matrix(rows, cols, idx + 1, plot_info)
        elif plot_type == 'graph':
            _draw_graph(rows, cols, idx + 1, plot_info)
        elif plot_type == 'drone_replay':
            _draw_drone_replay(rows, cols, idx + 1, plot_info)
        elif plot_type == 'trajectory_3d':
            _draw_trajectory_3d(rows, cols, idx + 1, plot_info)
        elif plot_type == 'eigenvalue':
            _draw_eigenvalue_plot(rows, cols, idx + 1, plot_info)
        elif plot_type == 'step_response':
            _draw_step_response(rows, cols, idx + 1, plot_info)
        elif plot_type == 'bode':
            _draw_bode_plot(rows, cols, idx + 1, plot_info)
        elif plot_type == 'pole_zero':
            _draw_pole_zero_map(rows, cols, idx + 1, plot_info)
        elif plot_type == 'confidence_interval':
            _draw_confidence_interval(rows, cols, idx + 1, plot_info)

    plt.tight_layout()


def _draw_timeseries(rows, cols, idx, plot_info):
    """Draw a timeseries plot at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx)

    data = plot_info['data']
    title = plot_info.get('title', '')
    labels = plot_info['labels']
    ylabel = plot_info['ylabel']
    colors_list = plot_info['colors']
    markers = plot_info.get('markers', None)
    linewidth = plot_info.get('linewidth', LINE_WIDTH)
    linestyles = plot_info.get('linestyles', None)
    dt = plot_info.get('dt', None)

    n_series, n_points = data.shape

    # Convert time steps to seconds if dt is provided
    if dt is not None:
        time_axis = np.arange(n_points) * dt
    else:
        time_axis = np.arange(n_points)

    for i in range(n_series):
        color = colors_list[i % len(colors_list)]
        label = labels[i] if labels and i < len(labels) else f'Series {i+1}'
        marker = markers[i] if markers and i < len(markers) else None
        linestyle = linestyles[i] if linestyles and i < len(linestyles) else '-'
        ax.plot(time_axis, data[i, :], linewidth=linewidth, color=color, label=label,
                marker=marker, markevery=max(1, n_points//20), linestyle=linestyle)

    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)
    ax.set_xlabel('Time [s]', fontsize=FONT_SIZE_LABEL)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=FONT_SIZE_LABEL)
    ax.tick_params(labelsize=FONT_SIZE_TICK)
    ax.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)
    ax.grid(True)


def _draw_lineplot(rows, cols, idx, plot_info):
    """Draw a line plot with custom x-axis at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx)

    x_data = plot_info['x_data']
    y_data = plot_info['y_data']
    title = plot_info.get('title', '')
    labels = plot_info['labels']
    xlabel = plot_info['xlabel']
    ylabel = plot_info['ylabel']
    colors_list = plot_info['colors']
    markers = plot_info.get('markers', None)
    linewidth = plot_info.get('linewidth', LINE_WIDTH)

    n_series = y_data.shape[0]

    for i in range(n_series):
        color = colors_list[i % len(colors_list)]
        label = labels[i] if labels and i < len(labels) else f'Series {i+1}'
        marker = markers[i] if markers and i < len(markers) else 'x'
        n_points = x_data.shape[1]
        ax.plot(x_data[i, :], y_data[i, :], linewidth=linewidth, color=color, label=label,
                marker=marker, markevery=max(1, n_points//20))

    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=FONT_SIZE_LABEL)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=FONT_SIZE_LABEL)
    ax.tick_params(labelsize=FONT_SIZE_TICK)
    ax.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)
    ax.grid(True)


def _draw_stacked(rows, cols, idx, plot_info):
    """Draw stacked timeseries at the specified position."""
    data = plot_info['data']
    title = plot_info.get('title', '')
    labels = plot_info['labels']
    ylabel = plot_info['ylabel']
    colors_list = plot_info['colors']
    linewidth = plot_info.get('linewidth', LINE_WIDTH)
    dt = plot_info.get('dt', None)

    n_series, n_points = data.shape

    # Convert time steps to seconds if dt is provided
    if dt is not None:
        time_axis = np.arange(n_points) * dt
    else:
        time_axis = np.arange(n_points)

    # Calculate position in figure
    col_idx = (idx - 1) % cols
    row_idx = (idx - 1) // cols

    # Create gridspec for stacked subplots within this cell
    left = col_idx / cols + 0.02
    right = (col_idx + 1) / cols - 0.02
    bottom = 1 - (row_idx + 1) / rows + 0.02
    top = 1 - row_idx / rows - 0.02

    gs = _figure.add_gridspec(n_series, 1,
                              left=left, right=right,
                              bottom=bottom, top=top,
                              hspace=0.3)

    for i in range(n_series):
        ax = _figure.add_subplot(gs[i])
        color = colors_list[i % len(colors_list)]
        label = labels[i] if labels and i < len(labels) else f'Series {i+1}'

        ax.plot(time_axis, data[i, :], linewidth=linewidth, color=color, label=label)
        ax.set_ylabel(label, fontsize=FONT_SIZE_LABEL)
        ax.tick_params(labelsize=FONT_SIZE_TICK)
        ax.grid(True)
        ax.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)

        if i < n_series - 1:
            ax.set_xticks([])
        else:
            ax.set_xlabel('Time [s]', fontsize=FONT_SIZE_LABEL)

        if i == 0 and title:
            ax.set_title(title, fontsize=FONT_SIZE_TITLE)


def _draw_matrix(rows, cols, idx, plot_info):
    """Draw a matrix plot at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx)

    matrix = plot_info['data']
    title = plot_info.get('title', '')
    cmap = plot_info['cmap']
    show_colorbar = plot_info['colorbar']
    Nx = plot_info['Nx']
    xlabel = plot_info.get('xlabel', '')
    ylabel = plot_info.get('ylabel', '')

    im = ax.imshow(matrix, cmap=cmap, interpolation='none', aspect='auto')

    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)

    if xlabel:
        ax.set_xlabel(xlabel, fontsize=FONT_SIZE_LABEL)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=FONT_SIZE_LABEL)

    ax.tick_params(labelsize=FONT_SIZE_TICK)
    ax.grid(True, linestyle='--', alpha=0.3, which='both', color='white')

    # ---- Draw vertical block separators ----
    if Nx is not None:
        ncols = matrix.shape[1]
        for k in range(1, ncols // Nx):
            x_position = k * Nx - 0.5
            ax.axvline(x=x_position, color='black', linewidth=1.5)

    if show_colorbar:
        _figure.colorbar(im, ax=ax, fraction=0.046, pad=0.04)


def add_timeseries(data, title='', labels=None, ylabel='', colors_list=None, markers=None, linewidth=None, linestyles=None, dt=None):
    """
    Add a time-series plot to the figure.

    Parameters:
    -----------
    data : ndarray
        Data to plot (n_series x n_timepoints)
    title : str
        Plot title (optional, won't render if empty)
    labels : list of str, optional
        Labels for each series
    ylabel : str
        Y-axis label
    colors_list : list of str, optional
        Colors for each series (defaults to global COLORS)
    markers : list of str, optional
        Marker styles for each series (defaults to 'x')
    linewidth : float, optional
        Line width in points (defaults to global LINE_WIDTH)
    linestyles : list of str, optional
        Line styles for each series (e.g., '-', '--', '-.', ':')
    dt : float, optional
        Time step in seconds. If provided, x-axis shows time in seconds
    """
    _init_figure()

    # Default colors
    if colors_list is None:
        colors_list = COLORS

    # Handle 1D data
    if data.ndim == 1:
        data = data.reshape(1, -1)

    # Store plot info
    _plots.append({
        'type': 'timeseries',
        'data': data.copy(),
        'title': title,
        'labels': labels,
        'ylabel': ylabel,
        'colors': colors_list,
        'markers': markers,
        'linewidth': linewidth if linewidth is not None else LINE_WIDTH,
        'linestyles': linestyles,
        'dt': dt
    })

    _redraw_all()


def add_lineplot(x_data, y_data, title='', labels=None, xlabel='', ylabel='', colors_list=None, markers=None, linewidth=None):
    """
    Add a line plot with custom x-axis data to the figure.

    Parameters:
    -----------
    x_data : ndarray
        X-axis data (1D array or n_series x n_points for per-series x values)
    y_data : ndarray
        Y-axis data (n_series x n_points)
    title : str
        Plot title (optional, won't render if empty)
    labels : list of str, optional
        Labels for each series
    xlabel : str
        X-axis label
    ylabel : str
        Y-axis label
    colors_list : list of str, optional
        Colors for each series (defaults to global COLORS)
    markers : list of str, optional
        Marker styles for each series (defaults to 'x')
    linewidth : float, optional
        Line width in points (defaults to global LINE_WIDTH)
    """
    _init_figure()

    # Default colors
    if colors_list is None:
        colors_list = COLORS

    # Handle 1D y_data
    if y_data.ndim == 1:
        y_data = y_data.reshape(1, -1)

    # Handle 1D x_data (shared across all series)
    if x_data.ndim == 1:
        x_data = np.tile(x_data, (y_data.shape[0], 1))

    # Store plot info
    _plots.append({
        'type': 'lineplot',
        'x_data': x_data.copy(),
        'y_data': y_data.copy(),
        'title': title,
        'labels': labels,
        'xlabel': xlabel,
        'ylabel': ylabel,
        'colors': colors_list,
        'markers': markers,
        'linewidth': linewidth if linewidth is not None else LINE_WIDTH
    })

    _redraw_all()


def add_stacked_timeseries(data, title='', labels=None, ylabel='', linewidth=None, dt=None):
    """
    Add stacked subplots for each series.

    Parameters:
    -----------
    data : ndarray
        Data to plot (n_series x n_timepoints)
    title : str
        Overall title (optional, won't render if empty)
    labels : list of str, optional
        Labels for each series
    ylabel : str
        Y-axis label (applied to all subplots)
    linewidth : float, optional
        Line width in points (defaults to global LINE_WIDTH)
    dt : float, optional
        Time step in seconds. If provided, x-axis shows time in seconds
    """
    _init_figure()

    # Handle 1D data
    if data.ndim == 1:
        data = data.reshape(1, -1)

    # Store plot info
    _plots.append({
        'type': 'stacked',
        'data': data.copy(),
        'title': title,
        'labels': labels,
        'ylabel': ylabel,
        'colors': COLORS,
        'linewidth': linewidth if linewidth is not None else LINE_WIDTH,
        'dt': dt
    })

    _redraw_all()


def add_matrix(matrix, title='', cmap='viridis', colorbar=True, Nx=None, xlabel='', ylabel=''):
    """
    Add a matrix visualization to the figure.

    Parameters:
    -----------
    matrix : ndarray
        2D matrix to visualize
    title : str
        Plot title
    cmap : str
        Colormap name
    colorbar : bool
        Whether to show colorbar
    Nx : int, optional
        Number of columns per block (draws vertical separators)
    xlabel : str
        X-axis label
    ylabel : str
        Y-axis label
    """
    _init_figure()

    # Store plot info
    _plots.append({
        'type': 'matrix',
        'data': matrix.copy(),
        'title': title,
        'cmap': cmap,
        'colorbar': colorbar,
        'Nx': Nx,
        'xlabel': xlabel,
        'ylabel': ylabel
    })

    _redraw_all()


def add_graph(adjacency_matrix, title='Graph', layout='spring', 
              node_colors=None, edge_color='#666666', node_size=500,
              show_weights=False, node_labels=None):
    """
    Add a graph visualization to the figure.
    
    Parameters:
    -----------
    adjacency_matrix : ndarray
        Adjacency matrix (n_nodes x n_nodes). Non-zero values represent edges.
    title : str
        Plot title
    layout : str
        Layout algorithm: 'spring', 'circular', 'kamada_kawai', 'spectral', 'shell', etc.
    node_colors : str or list, optional
        Single color or list of colors for each node. If None, uses default color.
    edge_color : str
        Color for edges
    node_size : int
        Size of nodes
    show_weights : bool
        Whether to display edge weights
    node_labels : list of str, optional
        Custom labels for nodes (supports LaTeX, e.g., r'$\theta$', r'$\dot{\theta}$')
    """
    _init_figure()
    
    # Store plot info
    _plots.append({
        'type': 'graph',
        'data': adjacency_matrix.copy(),
        'title': title,
        'layout': layout,
        'node_colors': node_colors,
        'edge_color': edge_color,
        'node_size': node_size,
        'show_weights': show_weights,
        'node_labels': node_labels
    })
    
    _redraw_all()


def _draw_graph(rows, cols, idx, plot_info):
    """Draw a graph plot at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx)

    adj_matrix = plot_info['data']
    title = plot_info.get('title', '')
    layout = plot_info['layout']
    node_colors = plot_info['node_colors']
    edge_color = plot_info['edge_color']
    node_size = plot_info['node_size']
    show_weights = plot_info['show_weights']
    node_labels = plot_info['node_labels']

    # Create directed graph from adjacency matrix
    G = nx.DiGraph(adj_matrix)
    n_nodes = len(G.nodes())

    # Set default node colors if not provided
    if node_colors is None:
        node_colors = COLORS[0]

    # Choose layout
    layout_funcs = {
        'spring': nx.spring_layout,
        'circular': nx.circular_layout,
        'kamada_kawai': nx.kamada_kawai_layout,
        'spectral': nx.spectral_layout,
        'shell': nx.shell_layout,
        'random': nx.random_layout,
        'spiral': nx.spiral_layout,
    }

    layout_func = layout_funcs.get(layout, nx.spring_layout)
    try:
        pos = layout_func(G, seed=42)
    except TypeError:
        pos = layout_func(G)

    # Draw nodes
    nx.draw_networkx_nodes(G, pos, ax=ax, node_color=node_colors,
                          node_size=node_size, edgecolors='black', linewidths=2)

    # Create custom labels dictionary
    if node_labels is not None:
        labels_dict = {i: node_labels[i] if i < len(node_labels) else str(i)
                      for i in range(n_nodes)}
    else:
        labels_dict = {i: str(i) for i in range(n_nodes)}

    # Draw node labels (with LaTeX support)
    nx.draw_networkx_labels(G, pos, labels_dict, ax=ax,
                           font_color='white', font_weight='bold', font_size=FONT_SIZE_LABEL)

    # Separate self-loops from regular edges
    edges = [(u, v) for u, v in G.edges() if u != v]
    self_loops = [(u, v) for u, v in G.edges() if u == v]

    # Draw regular edges with arrows
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=edges, edge_color=edge_color,
                          arrows=True, arrowsize=20, arrowstyle='->',
                          width=1.5, alpha=0.7, connectionstyle='arc3,rad=0.1')

    # Draw edge labels for regular edges (if show_weights is True)
    if show_weights:
        edge_labels = {(u, v): f'{adj_matrix[u, v]:.2f}' for u, v in edges}
        nx.draw_networkx_edge_labels(G, pos, edge_labels, ax=ax, font_size=FONT_SIZE_TICK,
                                    bbox=dict(boxstyle='round,pad=0.2',
                                            facecolor='white', alpha=0.8, edgecolor='none'))

        # Draw self-loop labels next to nodes
        for u, v in self_loops:
            x, y = pos[u]
            ax.text(x + 0.15, y + 0.15, f'{adj_matrix[u, v]:.2f}',
                   fontsize=FONT_SIZE_TICK, ha='left', va='bottom',
                   bbox=dict(boxstyle='round,pad=0.2',
                           facecolor='yellow', alpha=0.8, edgecolor='black'))

    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)

    ax.axis('off')

def add_drone_replay(state_data, title='Drone Replay', trail_length=50, 
                    speed=1.0, interval=50):
    """
    Add an animated 3D drone visualization replaying state data.
    
    Parameters:
    -----------
    state_data : ndarray
        State vector time series (12 x n_timepoints)
        Expected order: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
    title : str
        Plot title
    trail_length : int
        Length of position trail to show
    speed : float
        Playback speed multiplier
    interval : int
        Animation interval in milliseconds
    """
    
    _init_figure()
    
    _plots.append({
        'type': 'drone_replay',
        'data': state_data.copy(),
        'title': title,
        'trail_length': trail_length,
        'speed': speed,
        'interval': interval
    })
    
    _redraw_all()


def _draw_drone_replay(rows, cols, idx, plot_info):
    """Draw animated 3D drone at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx, projection='3d')

    state_data = plot_info['data']
    title = plot_info.get('title', '')
    trail_length = plot_info['trail_length']
    interval = plot_info['interval']

    # Extract position and orientation
    # State order: [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
    x, y, z = state_data[0], state_data[1], state_data[2]
    roll, pitch, yaw = state_data[6], state_data[7], state_data[8]

    n_points = len(x)

    # Set up the plot
    ax.set_xlim([x.min()-1, x.max()+1])
    ax.set_ylim([y.min()-1, y.max()+1])
    ax.set_zlim([z.min()-1, z.max()+1])
    ax.invert_zaxis()
    ax.set_xlabel('X', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Y', fontsize=FONT_SIZE_LABEL)
    ax.set_zlabel('Z (up)', fontsize=FONT_SIZE_LABEL)
    ax.tick_params(labelsize=FONT_SIZE_TICK)
    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)
    
    # Initialize drone components
    drone_body, = ax.plot([], [], [], 'ko', markersize=10)
    arm1, = ax.plot([], [], [], 'r-', linewidth=3)
    arm2, = ax.plot([], [], [], 'b-', linewidth=3)
    trail, = ax.plot([], [], [], 'gray', alpha=0.5, linewidth=1)
    
    # Rotation matrix - adjusted for NED to visualization frame
    def rotation_matrix(r, p, y):
        # Invert roll and pitch for Z-down to Z-up conversion
        Rz = np.array([[np.cos(y), -np.sin(y), 0],
                       [np.sin(y), np.cos(y), 0],
                       [0, 0, 1]])
        Ry = np.array([[np.cos(-p), 0, np.sin(-p)],  # NEGATE PITCH
                       [0, 1, 0],
                       [-np.sin(-p), 0, np.cos(-p)]])
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(-r), -np.sin(-r)],  # NEGATE ROLL
                       [0, np.sin(-r), np.cos(-r)]])
        return Rz @ Ry @ Rx
    
    def update(frame):
        # Get current state
        pos = np.array([x[frame], y[frame], z[frame]])
        R = rotation_matrix(-1 * pitch[frame], roll[frame], yaw[frame]) # This is a hacky fix
        
        # Drone arms (simple X configuration)
        arm_len = 0.3
        arms_body = np.array([
            [-arm_len, arm_len, 0],
            [arm_len, -arm_len, 0],
            [-arm_len, -arm_len, 0],
            [arm_len, arm_len, 0]
        ]).T
        
        arms_world = R @ arms_body + pos[:, np.newaxis]
        
        # Update drone
        drone_body.set_data([pos[0]], [pos[1]])
        drone_body.set_3d_properties([pos[2]])
        
        arm1.set_data([arms_world[0, 0], arms_world[0, 1]], 
                      [arms_world[1, 0], arms_world[1, 1]])
        arm1.set_3d_properties([arms_world[2, 0], arms_world[2, 1]])
        
        arm2.set_data([arms_world[0, 2], arms_world[0, 3]], 
                      [arms_world[1, 2], arms_world[1, 3]])
        arm2.set_3d_properties([arms_world[2, 2], arms_world[2, 3]])
        
        # Update trail
        start = max(0, frame - trail_length)
        trail.set_data(x[start:frame+1], y[start:frame+1])
        trail.set_3d_properties(z[start:frame+1])
        
        return drone_body, arm1, arm2, trail
    
    anim = FuncAnimation(_figure, update, frames=n_points, 
                        interval=interval, blit=True, repeat=True)
    
    # Store animation to prevent garbage collection
    if not hasattr(_figure, '_animations'):
        _figure._animations = []
    _figure._animations.append(anim)


def show_plots():
    """Display all added plots and reset the figure."""
    global _figure, _plots
    
    if _figure is not None:
        plt.tight_layout()
        plt.show(block=True)

        # Reset for next use
        plt.close(_figure)
        _figure = None
        _plots = []


def clear_plots():
    """Clear all plots without showing."""
    global _figure, _plots

    if _figure is not None:
        plt.close(_figure)

    _figure = None
    _plots = []


def save_plots_to_pdf(run_name: str = "default", output_dir: str = None):
    """
    Save all current plots to individual PDF files.
    Creates: <output_dir>/YYYY-MM-DD-HH-MM-SS-<run_name>/<plot-title>.pdf for each plot
    Args:
        run_name: Identifier for this run (used in folder name)
        output_dir: Base directory for output (defaults to ./data)
    """
    import os
    from datetime import datetime
    global _plots, _figure
    if not _plots:
        print("No plots to save")
        return

    # Use specified output directory or default to ./data
    base_dir = output_dir if output_dir is not None else "./data"
    datetime_str = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    folder = f"{base_dir}/{datetime_str}-{run_name}"
    os.makedirs(folder, exist_ok=True)
    # Save the original figure reference
    original_figure = _figure
    for idx, plot_info in enumerate(_plots):
        plot_type = plot_info['type']
        title = plot_info.get('title', '')

        # Build descriptive filename
        if title:
            # Sanitize title for filename
            safe_title = title.replace(' ', '_').replace('/', '-').replace('|', '-')
            safe_title = safe_title.replace('(', '').replace(')', '').replace(':', '')
            filename = f"{folder}/{run_name}_{plot_type}_{idx}_{safe_title}.pdf"
        else:
            filename = f"{folder}/{run_name}_{plot_type}_{idx}.pdf"
        # Skip animations in PDF export
        if plot_info['type'] == 'drone_replay':
            print(f"Skipping animation plot: {title}")
            continue
        # Create temporary figure for this plot only
        _figure = plt.figure(figsize=(10, 8))
        # Redraw single plot
        plot_type = plot_info['type']
        if plot_type == 'timeseries':
            _draw_timeseries(1, 1, 1, plot_info)
        elif plot_type == 'lineplot':
            _draw_lineplot(1, 1, 1, plot_info)
        elif plot_type == 'matrix':
            _draw_matrix(1, 1, 1, plot_info)
        elif plot_type == 'trajectory_3d':
            _draw_trajectory_3d(1, 1, 1, plot_info)
        elif plot_type == 'stacked':
            _draw_stacked(1, 1, 1, plot_info)
        elif plot_type == 'graph':
            _draw_graph(1, 1, 1, plot_info)
        elif plot_type == 'eigenvalue':
            _draw_eigenvalue_plot(1, 1, 1, plot_info)
        elif plot_type == 'step_response':
            _draw_step_response(1, 1, 1, plot_info)
        elif plot_type == 'bode':
            _draw_bode_plot(1, 1, 1, plot_info)
        elif plot_type == 'pole_zero':
            _draw_pole_zero_map(1, 1, 1, plot_info)
        elif plot_type == 'confidence_interval':
            _draw_confidence_interval(1, 1, 1, plot_info)
        _figure.savefig(filename, format='pdf', bbox_inches='tight')
        plt.close(_figure)
        print(f"Saved: {filename}")
    # Restore original figure and redraw all plots
    _figure = original_figure
    _redraw_all()


def add_trajectory_3d(traj: np.ndarray, title: str = '',
                     pos_indices: list = None,
                     show_start_end: bool = True):
    """
    Add standalone 3D trajectory plot.

    Args:
        traj: Trajectory data (Nx x T)
        title: Plot title (optional, won't render if empty)
        pos_indices: Indices for [x, y, z] positions (default: [0, 1, 2])
        show_start_end: Show start (green) and end (red) markers
    """
    _init_figure()

    if pos_indices is None:
        pos_indices = [0, 1, 2]

    _plots.append({
        'type': 'trajectory_3d',
        'data': traj.copy(),
        'title': title,
        'pos_indices': pos_indices,
        'show_start_end': show_start_end
    })

    _redraw_all()


def add_eigenvalue_plot(A_matrices, title='', labels=None, dt=None, show_unit_circle=True):
    """
    Add an eigenvalue plot (pole plot) for control systems analysis.

    This plot shows eigenvalues in the complex plane, which is critical for
    analyzing system stability and dynamic behavior.

    Parameters:
    -----------
    A_matrices : ndarray or list of ndarrays
        System matrix or list of system matrices. Each should be (n x n).
        For discrete-time systems, eigenvalues inside unit circle are stable.
        For continuous-time systems, eigenvalues in left half-plane are stable.
    title : str
        Plot title (optional, won't render if empty)
    labels : list of str, optional
        Labels for each system matrix
    dt : float, optional
        Sample time. If provided, shows unit circle (discrete-time).
        If None, shows imaginary axis (continuous-time).
    show_unit_circle : bool
        Whether to show stability boundary (unit circle for discrete, imaginary axis for continuous)
    """
    _init_figure()

    # Handle single matrix
    if isinstance(A_matrices, np.ndarray) and A_matrices.ndim == 2:
        A_matrices = [A_matrices]

    _plots.append({
        'type': 'eigenvalue',
        'data': A_matrices,
        'title': title,
        'labels': labels,
        'dt': dt,
        'show_unit_circle': show_unit_circle
    })

    _redraw_all()


def _draw_eigenvalue_plot(rows, cols, idx, plot_info):
    """Draw eigenvalue plot at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx)

    A_matrices = plot_info['data']
    title = plot_info.get('title', '')
    labels = plot_info['labels']
    dt = plot_info['dt']
    show_unit_circle = plot_info['show_unit_circle']

    # Compute eigenvalues for each matrix
    for i, A in enumerate(A_matrices):
        eigenvalues = np.linalg.eigvals(A)
        color = COLORS[i % len(COLORS)]
        label = labels[i] if labels and i < len(labels) else f'System {i+1}'

        # Plot eigenvalues
        ax.scatter(eigenvalues.real, eigenvalues.imag,
                  s=80, color=color, marker='x', linewidths=2,
                  label=label, zorder=3)

    # Add stability boundary
    if show_unit_circle:
        if dt is not None:  # Discrete-time system
            # Draw unit circle
            theta = np.linspace(0, 2*np.pi, 200)
            ax.plot(np.cos(theta), np.sin(theta), 'k--', linewidth=1,
                   alpha=0.5, label='Stability boundary', zorder=1)
            ax.set_xlabel('Real Part', fontsize=FONT_SIZE_LABEL)
            ax.set_ylabel('Imaginary Part', fontsize=FONT_SIZE_LABEL)
        else:  # Continuous-time system
            # Draw imaginary axis
            ylim = ax.get_ylim()
            ax.axvline(x=0, color='k', linestyle='--', linewidth=1,
                      alpha=0.5, label='Stability boundary', zorder=1)
            ax.set_xlabel('Real Part', fontsize=FONT_SIZE_LABEL)
            ax.set_ylabel('Imaginary Part', fontsize=FONT_SIZE_LABEL)

    # Make plot square for better visualization
    ax.set_aspect('equal', adjustable='box')
    ax.axhline(y=0, color=AXES_COLOR, linewidth=0.5, alpha=0.3)
    ax.axvline(x=0, color=AXES_COLOR, linewidth=0.5, alpha=0.3)
    ax.grid(True)
    ax.tick_params(labelsize=FONT_SIZE_TICK)
    ax.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)

    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)


def add_step_response(systems, title='', labels=None, dt=0.01, T_final=10):
    """
    Add step response plot for control systems.

    Shows the output response to a unit step input, useful for understanding
    rise time, settling time, overshoot, and steady-state behavior.

    Parameters:
    -----------
    systems : list of tuples
        List of (A, B, C, D) tuples representing state-space systems.
        Can also be a single tuple for one system.
    title : str
        Plot title (optional, won't render if empty)
    labels : list of str, optional
        Labels for each system
    dt : float
        Sample time for simulation
    T_final : float
        Final simulation time
    """
    _init_figure()

    # Handle single system
    if isinstance(systems, tuple) and len(systems) == 4:
        systems = [systems]

    _plots.append({
        'type': 'step_response',
        'data': systems,
        'title': title,
        'labels': labels,
        'dt': dt,
        'T_final': T_final
    })

    _redraw_all()


def _draw_step_response(rows, cols, idx, plot_info):
    """Draw step response at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx)

    systems = plot_info['data']
    title = plot_info.get('title', '')
    labels = plot_info['labels']
    dt = plot_info['dt']
    T_final = plot_info['T_final']

    t = np.arange(0, T_final, dt)
    n_steps = len(t)

    for i, (A, B, C, D) in enumerate(systems):
        # Initialize state
        x = np.zeros((A.shape[0], 1))
        y_out = np.zeros(n_steps)

        # Simulate step response
        u = 1.0  # Unit step input
        for k in range(n_steps):
            y_out[k] = (C @ x + D * u).flatten()[0]
            x = A @ x + B * u

        color = COLORS[i % len(COLORS)]
        label = labels[i] if labels and i < len(labels) else f'System {i+1}'

        ax.plot(t, y_out, linewidth=LINE_WIDTH, color=color, label=label)

    ax.set_xlabel('Time [s]', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Output', fontsize=FONT_SIZE_LABEL)
    ax.tick_params(labelsize=FONT_SIZE_TICK)
    ax.grid(True)
    ax.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)

    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)


def add_bode_plot(systems, title='', labels=None, w_min=0.01, w_max=100, n_points=200):
    """
    Add Bode plot (magnitude and phase vs frequency) for control systems.

    Critical for frequency domain analysis, showing gain and phase margins,
    bandwidth, and resonant peaks.

    Parameters:
    -----------
    systems : list of tuples
        List of (A, B, C, D) tuples representing state-space systems.
        Can also be a single tuple for one system.
    title : str
        Plot title (optional, won't render if empty)
    labels : list of str, optional
        Labels for each system
    w_min : float
        Minimum frequency (rad/s)
    w_max : float
        Maximum frequency (rad/s)
    n_points : int
        Number of frequency points
    """
    _init_figure()

    # Handle single system
    if isinstance(systems, tuple) and len(systems) == 4:
        systems = [systems]

    _plots.append({
        'type': 'bode',
        'data': systems,
        'title': title,
        'labels': labels,
        'w_min': w_min,
        'w_max': w_max,
        'n_points': n_points
    })

    _redraw_all()


def _draw_bode_plot(rows, cols, idx, plot_info):
    """Draw Bode plot at the specified position."""
    # This takes two subplot slots (magnitude and phase)
    systems = plot_info['data']
    title = plot_info.get('title', '')
    labels = plot_info['labels']
    w_min = plot_info['w_min']
    w_max = plot_info['w_max']
    n_points = plot_info['n_points']

    # Create frequency vector
    w = np.logspace(np.log10(w_min), np.log10(w_max), n_points)

    # Calculate position in grid
    col_idx = (idx - 1) % cols
    row_idx = (idx - 1) // rows

    # Create two vertically stacked subplots
    left = col_idx / cols + 0.02
    right = (col_idx + 1) / cols - 0.02
    bottom = 1 - (row_idx + 1) / rows + 0.02
    top = 1 - row_idx / rows - 0.02

    gs = _figure.add_gridspec(2, 1, left=left, right=right, bottom=bottom, top=top, hspace=0.1)

    ax_mag = _figure.add_subplot(gs[0])
    ax_phase = _figure.add_subplot(gs[1])

    for i, (A, B, C, D) in enumerate(systems):
        mag_db = np.zeros(n_points)
        phase_deg = np.zeros(n_points)

        # Compute frequency response
        for k, w_k in enumerate(w):
            # Frequency response: H(jω) = C(jωI - A)^(-1)B + D
            s = 1j * w_k
            I = np.eye(A.shape[0])
            try:
                H = C @ np.linalg.solve(s * I - A, B) + D
                mag_db[k] = 20 * np.log10(np.abs(H.item()))
                phase_deg[k] = np.angle(H.item(), deg=True)
            except np.linalg.LinAlgError:
                mag_db[k] = np.nan
                phase_deg[k] = np.nan

        color = COLORS[i % len(COLORS)]
        label = labels[i] if labels and i < len(labels) else f'System {i+1}'

        ax_mag.semilogx(w, mag_db, linewidth=LINE_WIDTH, color=color, label=label)
        ax_phase.semilogx(w, phase_deg, linewidth=LINE_WIDTH, color=color, label=label)

    # Magnitude plot
    ax_mag.set_ylabel('Magnitude [dB]', fontsize=FONT_SIZE_LABEL)
    ax_mag.tick_params(labelsize=FONT_SIZE_TICK)
    ax_mag.grid(True, which='both')
    ax_mag.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)
    ax_mag.set_xticklabels([])  # Remove x-tick labels from top plot

    if title:
        ax_mag.set_title(title, fontsize=FONT_SIZE_TITLE)

    # Phase plot
    ax_phase.set_xlabel('Frequency [rad/s]', fontsize=FONT_SIZE_LABEL)
    ax_phase.set_ylabel('Phase [deg]', fontsize=FONT_SIZE_LABEL)
    ax_phase.tick_params(labelsize=FONT_SIZE_TICK)
    ax_phase.grid(True, which='both')


def add_pole_zero_map(systems, title='', labels=None):
    """
    Add pole-zero map for control systems.

    Shows both poles (eigenvalues of A) and zeros (transmission zeros) in
    the complex plane, useful for understanding system dynamics and control design.

    Parameters:
    -----------
    systems : list of tuples
        List of (A, B, C, D) tuples representing state-space systems.
        Can also be a single tuple for one system.
    title : str
        Plot title (optional, won't render if empty)
    labels : list of str, optional
        Labels for each system
    """
    _init_figure()

    # Handle single system
    if isinstance(systems, tuple) and len(systems) == 4:
        systems = [systems]

    _plots.append({
        'type': 'pole_zero',
        'data': systems,
        'title': title,
        'labels': labels
    })

    _redraw_all()


def _draw_pole_zero_map(rows, cols, idx, plot_info):
    """Draw pole-zero map at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx)

    systems = plot_info['data']
    title = plot_info.get('title', '')
    labels = plot_info['labels']

    for i, (A, B, C, D) in enumerate(systems):
        # Compute poles (eigenvalues of A)
        poles = np.linalg.eigvals(A)

        color = COLORS[i % len(COLORS)]
        label = labels[i] if labels and i < len(labels) else f'System {i+1}'

        # Plot poles with 'x' marker
        ax.scatter(poles.real, poles.imag, s=100, color=color,
                  marker='x', linewidths=2.5, label=f'{label} (poles)', zorder=3)

        # Compute zeros (more complex, requires transmission zeros)
        # For SISO: zeros are roots of det([A-sI, B; C, D])
        # Simplified: skip if system is MIMO or computation fails
        try:
            if B.shape[1] == 1 and C.shape[0] == 1:  # SISO system
                n = A.shape[0]
                s = np.polynomial.polynomial.polyroots
                # Transmission zeros computation (simplified)
                # This is a placeholder - full implementation requires more complex computation
        except:
            pass

    # Add stability boundary (imaginary axis for continuous-time)
    ax.axvline(x=0, color='k', linestyle='--', linewidth=1, alpha=0.5, zorder=1)

    ax.set_aspect('equal', adjustable='box')
    ax.axhline(y=0, color=AXES_COLOR, linewidth=0.5, alpha=0.3)
    ax.axvline(x=0, color=AXES_COLOR, linewidth=0.5, alpha=0.3)
    ax.grid(True)
    ax.set_xlabel('Real Part', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Imaginary Part', fontsize=FONT_SIZE_LABEL)
    ax.tick_params(labelsize=FONT_SIZE_TICK)
    ax.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)

    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)


def _draw_trajectory_3d(rows, cols, idx, plot_info):
    """Draw 3D trajectory at specified position."""
    ax = _figure.add_subplot(rows, cols, idx, projection='3d')

    traj = plot_info['data']
    pos_indices = plot_info['pos_indices']
    title = plot_info.get('title', '')

    x = traj[pos_indices[0], :]
    y = traj[pos_indices[1], :]
    z = traj[pos_indices[2], :]

    ax.plot(x, y, z, color=COLORS[0], linewidth=LINE_WIDTH, label='Trajectory')

    if plot_info['show_start_end']:
        ax.scatter(x[0], y[0], z[0], c='g', s=100, marker='o', label='Start', edgecolors='black')
        ax.scatter(x[-1], y[-1], z[-1], c='r', s=100, marker='x', label='End', linewidths=3)

    ax.set_xlabel('X [m]', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Y [m]', fontsize=FONT_SIZE_LABEL)
    ax.set_zlabel('Z [m]', fontsize=FONT_SIZE_LABEL)
    ax.tick_params(labelsize=FONT_SIZE_TICK)
    ax.invert_zaxis()  # NED frame: invert Z for intuitive display
    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)
    ax.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)
    ax.grid(True)


def add_confidence_interval(data, title='', ylabel='', metric='error', dt=None,
                           sigma=2.0, color=None, show_mean=True, show_individual=False):
    """
    Add a confidence interval plot showing mean ± sigma bounds across multiple runs.

    This plot is commonly used to compare trajectory tracking performance across
    different control methods by visualizing the mean tracking error/cost and its
    variability over time.

    Parameters:
    -----------
    data : ndarray
        Data from multiple runs (n_runs x n_timepoints) or (n_runs x n_series x n_timepoints).
        For tracking error: pass the error magnitude over time for each run.
        For cost: pass the cumulative or instantaneous cost for each run.
    title : str
        Plot title (optional, won't render if empty)
    ylabel : str
        Y-axis label (e.g., 'Tracking Error [m]', 'Cost', 'RMSE [m]')
    metric : str
        Type of metric being plotted ('error', 'cost', 'rmse', 'custom')
        This is used for automatic ylabel if ylabel is not provided
    dt : float, optional
        Time step in seconds. If provided, x-axis shows time in seconds
    sigma : float
        Number of standard deviations for confidence interval (default: 2.0 for ~95%)
    color : str, optional
        Color for mean line and confidence band (defaults to first COLORS entry)
    show_mean : bool
        Whether to show the mean line (default: True)
    show_individual : bool
        Whether to show individual runs as faint lines (default: False)
    """
    _init_figure()

    # Handle different data shapes
    if data.ndim == 2:
        # (n_runs x n_timepoints)
        pass
    elif data.ndim == 3:
        # (n_runs x n_series x n_timepoints) - compute norm across series
        data = np.linalg.norm(data, axis=1)
    else:
        raise ValueError(f"Data must be 2D or 3D, got shape {data.shape}")

    # Default color
    if color is None:
        color = COLORS[0]

    # Auto ylabel based on metric
    if not ylabel:
        metric_labels = {
            'error': 'Tracking Error [m]',
            'cost': 'Cost',
            'rmse': 'RMSE [m]',
            'custom': 'Value'
        }
        ylabel = metric_labels.get(metric, 'Value')

    # Store plot info
    _plots.append({
        'type': 'confidence_interval',
        'data': data.copy(),
        'title': title,
        'ylabel': ylabel,
        'dt': dt,
        'sigma': sigma,
        'color': color,
        'show_mean': show_mean,
        'show_individual': show_individual
    })

    _redraw_all()


def _draw_confidence_interval(rows, cols, idx, plot_info):
    """Draw confidence interval plot at the specified position."""
    ax = _figure.add_subplot(rows, cols, idx)

    data = plot_info['data']
    title = plot_info.get('title', '')
    ylabel = plot_info['ylabel']
    dt = plot_info.get('dt', None)
    sigma = plot_info['sigma']
    color = plot_info['color']
    show_mean = plot_info['show_mean']
    show_individual = plot_info['show_individual']

    n_runs, n_points = data.shape

    # Convert time steps to seconds if dt is provided
    if dt is not None:
        time_axis = np.arange(n_points) * dt
    else:
        time_axis = np.arange(n_points)

    # Compute statistics across runs
    mean = np.mean(data, axis=0)
    std = np.std(data, axis=0)
    upper_bound = mean + sigma * std
    lower_bound = mean - sigma * std

    # Plot individual runs if requested (faint lines in background)
    if show_individual:
        for i in range(n_runs):
            ax.plot(time_axis, data[i, :], linewidth=0.5, color=color,
                   alpha=0.15, zorder=1)

    # Plot confidence interval (filled region)
    ax.fill_between(time_axis, lower_bound, upper_bound,
                    color=color, alpha=0.3, label=f'±{sigma}σ', zorder=2)

    # Plot mean line
    if show_mean:
        ax.plot(time_axis, mean, linewidth=LINE_WIDTH, color=color,
               label=f'Mean (n={n_runs})', zorder=3)

    # Formatting
    if title:
        ax.set_title(title, fontsize=FONT_SIZE_TITLE)
    ax.set_xlabel('Time [s]', fontsize=FONT_SIZE_LABEL)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=FONT_SIZE_LABEL)
    ax.tick_params(labelsize=FONT_SIZE_TICK)
    ax.legend(fontsize=FONT_SIZE_LEGEND, frameon=False)
    ax.grid(True)


# Example usage
if __name__ == "__main__":
    # Create sample data
    ts1 = np.random.randn(3, 100).cumsum(axis=1)
    ts2 = np.sin(np.linspace(0, 4*np.pi, 100)) + np.random.randn(100)*0.1
    matrix1 = np.random.randn(10, 10)
    matrix2 = np.eye(8) * 5
    
    # Add various plots
    add_timeseries(ts1, title='Random Walk', labels=['A', 'B', 'C'])
    add_timeseries(ts2, title='Sine Wave', ylabel='Amplitude')
    add_matrix(matrix1, title='Random Matrix', cmap='plasma')
    add_matrix(matrix2, title='Diagonal Matrix', cmap='viridis')
    add_stacked_timeseries(ts1, title='Stacked View', labels=['A', 'B', 'C'])
    
    # Show all plots
    show_plots()
