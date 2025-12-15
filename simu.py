#!/usr/bin/env python3
import os
import glob
import subprocess
import signal
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import yaml
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# Try to use ament to find package, fallback to workspace path
try:
    from ament_index_python.packages import get_package_share_directory
    PKG = 'multirobot_gz_simulator'
    PKG_SHARE = get_package_share_directory(PKG)
    logger.info(f"Package share directory: {PKG_SHARE}")
except Exception as e:
    logger.warning(f"Could not find package via ament: {e}. Using fallback path.")
    PKG = None
    PKG_SHARE = os.path.join(os.path.dirname(__file__), '..', '..')

DEFAULT_CONFIG_DIR = os.path.join(PKG_SHARE, 'config')
DEFAULT_WORLDS_DIR = os.path.join(PKG_SHARE, 'worlds')
DEFAULT_MODELS_DIR = os.path.join(PKG_SHARE, 'models')

def find_worlds():
    """Find all world files (.sdf, .world) in the package."""
    pats = []
    if os.path.isdir(DEFAULT_WORLDS_DIR):
        # Look for .sdf files (standard Gazebo format)
        pats += glob.glob(os.path.join(DEFAULT_WORLDS_DIR, '*.sdf'))
        pats += glob.glob(os.path.join(DEFAULT_WORLDS_DIR, '*.world'))
    # Also search workspace recursively
    pats += glob.glob(os.path.join(PKG_SHARE, '**', '*.sdf'), recursive=True)
    pats += glob.glob(os.path.join(PKG_SHARE, '**', '*.world'), recursive=True)
    # Remove duplicates and sort
    return sorted(list(set([p for p in pats if os.path.isfile(p)])))

def find_model_types():
    """Find all available robot model types in the models directory."""
    types = []
    if os.path.isdir(DEFAULT_MODELS_DIR):
        for d in os.listdir(DEFAULT_MODELS_DIR):
            model_path = os.path.join(DEFAULT_MODELS_DIR, d)
            # Check if it's a directory with a model.sdf file
            if os.path.isdir(model_path) and os.path.exists(os.path.join(model_path, 'model.sdf')):
                types.append(d)
            elif os.path.isdir(model_path):
                logger.debug(f"Skipping {d}: no model.sdf found")
    if not types:
        logger.warning(f"No valid models found in {DEFAULT_MODELS_DIR}")
    return sorted(types)

def validate_config(config):
    """Validate the fleet configuration structure."""
    required_keys = ['world_file', 'world_name', 'robot_ns', 'robots']
    for key in required_keys:
        if key not in config:
            raise ValueError(f"Missing required key: {key}")
    
    if not isinstance(config['robots'], list):
        raise ValueError("'robots' must be a list")
    
    for i, robot in enumerate(config['robots']):
        if not isinstance(robot, dict):
            raise ValueError(f"Robot {i} is not a dictionary")
        robot_keys = {'name', 'type', 'pose'}
        if not robot_keys.issubset(robot.keys()):
            raise ValueError(f"Robot {i} missing keys: {robot_keys - robot.keys()}")
        if not isinstance(robot['pose'], list) or len(robot['pose']) != 6:
            raise ValueError(f"Robot {i} pose must be a list of 6 floats")
    
    return True

class FleetGUI(tk.Tk):
    """GUI for creating and managing multi-robot fleet configurations."""
    
    def __init__(self):
        super().__init__()
        self.title("Fleet Config Creator")
        self.geometry("900x600")
        self.resizable(True, True)
        
        # Track launched processes
        self.launched_process = None
        
        # Setup cleanup on window close
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Load available worlds and models
        # keep both full paths and display names (basename) to show only names in the UI
        self.worlds_full = find_worlds()
        self.worlds = [os.path.basename(p) for p in self.worlds_full]
        self.model_types = find_model_types()
        self.robots = []
        
        if not self.worlds:
            logger.warning("No worlds found!")
        if not self.model_types:
            logger.warning("No robot models found!")
        
        self._build_ui()
    
    def on_closing(self):
        """Handle window closing and cleanup processes."""
        # If a simulator was launched, ask the user and terminate it cleanly
        try:
            if self.launched_process and self.launched_process.poll() is None:
                if messagebox.askyesno("Confirm", "A simulator is running. Terminate it?"):
                    self._terminate_processes()
                else:
                    return
        except Exception:
            # If any error checking the process, attempt cleanup anyway
            try:
                self._terminate_processes()
            except Exception:
                pass

        # Destroy the window and exit mainloop
        try:
            self.destroy()
        except Exception:
            try:
                self.quit()
            except Exception:
                pass
    
    def _terminate_processes(self):
        """Terminate all launched processes."""
        if self.launched_process:
            try:
                pid = self.launched_process.pid
                logger.info(f"Terminating process group for PID {pid}")
                # Try to terminate the whole process group
                try:
                    pgid = os.getpgid(pid)
                    os.killpg(pgid, signal.SIGTERM)
                except Exception:
                    # Fallback to terminating the single process
                    logger.debug("Could not kill process group, terminating single process")
                    try:
                        self.launched_process.terminate()
                    except Exception:
                        pass

                try:
                    # Wait up to 5 seconds for graceful termination
                    self.launched_process.wait(timeout=5)
                    logger.info("Process terminated gracefully")
                except subprocess.TimeoutExpired:
                    logger.warning("Process did not terminate, killing group")
                    try:
                        pgid = os.getpgid(pid)
                        os.killpg(pgid, signal.SIGKILL)
                    except Exception:
                        try:
                            self.launched_process.kill()
                        except Exception:
                            pass
                    try:
                        self.launched_process.wait()
                    except Exception:
                        pass
                    logger.info("Process killed")
                finally:
                    # Ensure any child processes are killed as well
                    try:
                        self._kill_descendants(pid)
                    except Exception:
                        logger.debug("Failed to kill descendants via fallback")
            except Exception as e:
                logger.error(f"Error terminating process: {e}")

        # Fallback: find any remaining `ros2 launch multirobot_gz_simulator` processes and kill their descendants
        try:
            out = subprocess.check_output("pgrep -f 'ros2 launch multirobot_gz_simulator' || true", shell=True, text=True)
            for line in out.splitlines():
                try:
                    p = int(line.strip())
                    logger.info(f"Cleaning up leftover ros2 launch PID {p}")
                    try:
                        self._kill_descendants(p)
                    except Exception:
                        logger.debug(f"Failed to kill descendants of {p}")
                    try:
                        os.kill(p, signal.SIGTERM)
                    except Exception:
                        pass
                except Exception:
                    continue
        except Exception:
            pass

        # Try pkill for common Gazebo/bridge binaries and relays as a last-resort
        pkill_patterns = [
            "gz sim",
            "parameter_bridge",
            "ros_gz_bridge",
            "ruby .*gz sim",
            "tf_relay",
        ]
        for pat in pkill_patterns:
            try:
                subprocess.run(f"pkill -f '{pat}' || true", shell=True)
            except Exception:
                pass
        # stop ros2 daemon to clear discovery and nodes/topics
        try:
            subprocess.run(["ros2", "daemon", "stop"], timeout=3)
        except Exception:
            pass
        # mark as terminated
        try:
            self.launched_process = None
        except Exception:
            pass
    
    def _build_ui(self):
        """Build the user interface."""
        # Top frame: World selection
        frame_top = ttk.LabelFrame(self, text="World Configuration", padding=8)
        frame_top.pack(fill="x", padx=8, pady=8)
        
        ttk.Label(frame_top, text="World:").grid(row=0, column=0, sticky="w")
        self.world_cb = ttk.Combobox(frame_top, values=self.worlds, width=70, state="readonly")
        if self.worlds:
            # Set the combobox to show the basename of the first world
            self.world_cb.set(self.worlds[0])
        self.world_cb.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(frame_top, text="Browse...", command=self.browse_world).grid(row=0, column=2, padx=4)
        frame_top.columnconfigure(1, weight=1)
        
        ttk.Label(frame_top, text="Fleet Namespace:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.ns_entry = ttk.Entry(frame_top)
        self.ns_entry.insert(0, "fleet")
        self.ns_entry.grid(row=1, column=1, sticky="ew", padx=4, pady=(6, 0))
        
        # Middle frame: Robot entry
        frame_mid = ttk.LabelFrame(self, text="Add Robot", padding=8)
        frame_mid.pack(fill="x", padx=8, pady=6)
        
        ttk.Label(frame_mid, text="Name:").grid(row=0, column=0, padx=4, pady=4, sticky="e")
        self.rname = ttk.Entry(frame_mid, width=16)
        self.rname.grid(row=0, column=1, padx=4)
        
        ttk.Label(frame_mid, text="Type:").grid(row=0, column=2, padx=4, sticky="e")
        self.rtype = ttk.Combobox(frame_mid, values=self.model_types, width=25, state="readonly")
        self.rtype.grid(row=0, column=3, padx=4)
        
        ttk.Label(frame_mid, text="Pose (x y z r p y):").grid(row=1, column=0, columnspan=2, padx=4, pady=4, sticky="nw")
        self.rpose = ttk.Entry(frame_mid, width=50)
        self.rpose.insert(0, "0 0 0 0 0 0")
        self.rpose.grid(row=1, column=2, columnspan=2, sticky="ew", padx=4)
        
        ttk.Button(frame_mid, text="Add Robot", command=self.add_robot).grid(row=0, column=4, padx=6, pady=4)
        frame_mid.columnconfigure(3, weight=1)
        # Update / delete controls for selected robot (side-by-side)
        ttk.Button(frame_mid, text="Update", command=self.update_robot).grid(row=0, column=5, padx=6, pady=4)
        ttk.Button(frame_mid, text="Delete Selected", command=self.delete_selected).grid(row=0, column=6, padx=6, pady=4)

        # Control buttons (toolbar at top of the list)
        frame_bot = ttk.Frame(self)
        frame_bot.pack(side="top", fill="x", padx=8, pady=8)
        # 'New Config' button removed per request
        ttk.Button(frame_bot, text="Load Config", command=self.read_config).pack(side="left", padx=4)        
        ttk.Button(frame_bot, text="Save Config", command=self.save_config).pack(side="left", padx=4)        
        # Clear Robots button removed per request
        ttk.Button(frame_bot, text="Quit", command=self.on_closing).pack(side="right", padx=4)
        ttk.Button(frame_bot, text="Kill Simulator", command=self.kill_simulator).pack(side="right", padx=4)        
        ttk.Button(frame_bot, text="Save & Launch", command=self.save_and_launch).pack(side="right", padx=4)

        # List frame: Show added robots
        frame_list = ttk.LabelFrame(self, text="Robot Fleet", padding=8)
        frame_list.pack(fill="both", expand=True, padx=8, pady=6)

        self.listbox = tk.Listbox(frame_list, height=10, font=("Courier", 9))
        self.listbox.pack(side="left", fill="both", expand=True)

        scrollbar = ttk.Scrollbar(frame_list, command=self.listbox.yview)
        scrollbar.pack(side="right", fill="y")
        self.listbox.config(yscrollcommand=scrollbar.set)
        # selection handling
        self.selected_robot_index = None
        self.listbox.bind('<<ListboxSelect>>', self.on_list_select)
        # Single click on an item will open the editor for that robot
        self.listbox.bind('<ButtonRelease-1>', self._on_list_click)
        self.listbox.bind('<Double-Button-1>', lambda e: self.edit_selected())

    def _get_descendants(self, pid):
        """Return a list of descendant PIDs for given pid (recursive)."""
        descendants = []
        try:
            out = subprocess.check_output(["pgrep", "-P", str(pid)], text=True).strip()
            if out:
                for line in out.splitlines():
                    try:
                        cpid = int(line.strip())
                        descendants.append(cpid)
                        descendants.extend(self._get_descendants(cpid))
                    except Exception:
                        continue
        except subprocess.CalledProcessError:
            # no children
            pass
        except Exception:
            pass
        return descendants

    def _kill_descendants(self, pid):
        """Terminate descendant processes of pid (SIGTERM then SIGKILL)."""
        children = self._get_descendants(pid)
        if not children:
            return
        logger.info(f"Killing descendant PIDs: {children}")
        # gentle first
        for c in children:
            try:
                os.kill(c, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1)
        # force kill remaining
        for c in children:
            try:
                os.kill(c, signal.SIGKILL)
            except Exception:
                pass

    def kill_simulator(self):
        """Run pkill commands to aggressively stop any running simulator processes."""
        if not messagebox.askyesno("Kill Simulator", "This will forcefully stop Gazebo and related bridges. Continue?"):
            return
        logger.info("Running aggressive simulator kill sequence")
        try:
            # stop ros2 launches for this package
            subprocess.run("pkill -f 'ros2.*launch.*multirobot_gz_simulator' || true", shell=True)
            time.sleep(1)
            # consolidated force-kill patterns
            force_patterns = ["gz sim", "parameter_bridge", "ros_gz_bridge", "ruby .*gz sim"]
            for pat in force_patterns:
                try:
                    subprocess.run(f"pkill -9 -f '{pat}' || true", shell=True)
                except Exception:
                    pass
            messagebox.showinfo("Killed", "Requested simulator processes have been terminated (if running)")
        except Exception as e:
            logger.error(f"Error running kill commands: {e}")
            messagebox.showerror("Error", f"Failed to kill simulator processes:\n{e}")

    def read_config(self):
        """Read and load an existing configuration file."""
        # Default target directory
        target = DEFAULT_CONFIG_DIR if os.path.isdir(DEFAULT_CONFIG_DIR) else os.getcwd()
        
        # Ask user to select config file
        cfg_file = filedialog.askopenfilename(
            title="Select configuration file to load",
            initialdir=target,
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if not cfg_file:
            return
        
        try:
            with open(cfg_file, "r") as f:
                cfg = yaml.safe_load(f)
            
            # Validate configuration
            validate_config(cfg)
            
            # Load configuration into UI
            self.load_config(cfg)
            logger.info(f"Configuration loaded from {cfg_file}")
            messagebox.showinfo("Success", f"Configuration loaded from\n{cfg_file}")
            
        except Exception as e:
            logger.error(f"Failed to load configuration: {e}")
            messagebox.showerror("Error", f"Failed to load configuration:\n{e}")
    
    def load_config(self, cfg):
        """Load configuration dictionary into the UI."""
        # Set world
        world_file = cfg.get('world_file', '')
        if world_file:
            # Try to find the world in the known full paths by basename
            found = False
            for full in self.worlds_full:
                if os.path.basename(full) == world_file or full.endswith(world_file):
                    # set combobox display to basename
                    self.world_cb.set(os.path.basename(full))
                    found = True
                    break
            if not found:
                # fallback: show just the filename (user can browse to correct path)
                self.world_cb.set(world_file)
        
        # Set namespace
        ns = cfg.get('robot_ns', 'robot')
        self.ns_entry.delete(0, 'end')
        self.ns_entry.insert(0, ns)
        
        # Clear and load robots
        self.robots = []
        self.listbox.delete(0, 'end')
        
        for robot in cfg.get('robots', []):
            self.robots.append(robot)
            name = robot.get('name', 'unknown')
            typ = robot.get('type', 'unknown')
            pose = robot.get('pose', [0]*6)
            display_text = f"{name:15} | {typ:20} | pose: {[round(p, 2) for p in pose]}"
            self.listbox.insert("end", display_text)
        
        logger.info(f"Loaded {len(self.robots)} robots from configuration")
    


    def browse_world(self):
        """Browse and select a world file."""
        path = filedialog.askopenfilename(
            title="Select world file",
            initialdir=DEFAULT_WORLDS_DIR if os.path.isdir(DEFAULT_WORLDS_DIR) else os.getcwd(),
            filetypes=[("World files", "*.sdf *.world"), ("SDF files", "*.sdf"), ("All files", "*.*")]
        )
        if path:
            name = os.path.basename(path)
            # if new, add to lists and update combobox values
            if path not in self.worlds_full:
                self.worlds_full.append(path)
                self.worlds.append(name)
                self.world_cb['values'] = self.worlds
            self.world_cb.set(name)
            logger.info(f"Selected world: {path}")

    def add_robot(self):
        """Add a robot to the fleet configuration."""
        name = self.rname.get().strip()
        typ = self.rtype.get().strip()
        pose_str = self.rpose.get().strip()
        
        # Validate inputs
        if not name:
            messagebox.showerror("Error", "Robot name cannot be empty")
            return
        if not typ:
            messagebox.showerror("Error", "Robot type must be selected")
            return
        
        # Check for duplicate names (allow if updating the selected robot)
        if any(r['name'] == name for i, r in enumerate(self.robots) if i != (self.selected_robot_index if self.selected_robot_index is not None else -1)):
            messagebox.showerror("Error", f"Robot name '{name}' already exists")
            return
        
        # Parse pose
        try:
            pose = [float(x) for x in pose_str.split()]
            if len(pose) != 6:
                raise ValueError("Expected 6 values")
        except (ValueError, TypeError) as e:
            messagebox.showerror("Error", f"Invalid pose: must be 6 floats (x y z roll pitch yaw)\nError: {e}")
            return
        
        # Add robot to list
        robot = {'name': name, 'type': typ, 'pose': pose}
        self.robots.append(robot)
        display_text = f"{name:15} | {typ:20} | pose: {[round(p, 2) for p in pose]}"
        self.listbox.insert("end", display_text)
        logger.info(f"Added robot: {name} (type: {typ})")
        
        # Clear input fields
        self.rname.delete(0, 'end')
        self.rpose.delete(0, 'end')
        self.rpose.insert(0, "0 0 0 0 0 0")
        self.rtype.set("")
        self.rname.focus()

    def on_list_select(self, event=None):
        sel = self.listbox.curselection()
        if not sel:
            self.selected_robot_index = None
            return
        self.selected_robot_index = sel[0]

    def edit_selected(self):
        """Populate input fields with selected robot for editing."""
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showinfo("Edit Robot", "No robot selected")
            return
        idx = sel[0]
        robot = self.robots[idx]
        self.rname.delete(0, 'end')
        self.rname.insert(0, robot.get('name', ''))
        self.rtype.set(robot.get('type', ''))
        self.rpose.delete(0, 'end')
        self.rpose.insert(0, ' '.join(str(x) for x in robot.get('pose', [0]*6)))
        self.selected_robot_index = idx

    def update_robot(self):
        """Update the currently selected robot with values from the input fields."""
        if self.selected_robot_index is None:
            messagebox.showinfo("Update Robot", "No robot selected to update")
            return
        name = self.rname.get().strip()
        typ = self.rtype.get().strip()
        pose_str = self.rpose.get().strip()
        if not name or not typ:
            messagebox.showerror("Error", "Provide name and type")
            return
        try:
            pose = [float(x) for x in pose_str.split()]
            if len(pose) != 6:
                raise ValueError()
        except Exception:
            messagebox.showerror("Error", "Pose must be 6 floats: x y z r p y")
            return
        # check duplicate name excluding current
        for i, r in enumerate(self.robots):
            if i != self.selected_robot_index and r['name'] == name:
                messagebox.showerror("Error", f"Robot name '{name}' already exists")
                return
        # apply update
        self.robots[self.selected_robot_index] = {'name': name, 'type': typ, 'pose': pose}
        display_text = f"{name:15} | {typ:20} | pose: {[round(p, 2) for p in pose]}"
        self.listbox.delete(self.selected_robot_index)
        self.listbox.insert(self.selected_robot_index, display_text)
        logger.info(f"Updated robot at index {self.selected_robot_index}: {name}")
        # clear selection
        self.listbox.selection_clear(0, 'end')
        self.selected_robot_index = None

    def delete_selected(self):
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showinfo("Delete Robot", "No robot selected")
            return
        idx = sel[0]
        if not messagebox.askyesno("Delete Robot", f"Delete robot {self.robots[idx]['name']}?"):
            return
        self.listbox.delete(idx)
        del self.robots[idx]
        logger.info(f"Deleted robot at index {idx}")
        self.selected_robot_index = None

    def _on_list_click(self, event):
        """Handle single-click on listbox: if clicked on an item, select it and open editor."""
        try:
            # nearest returns the index of the nearest item to the y coordinate
            idx = self.listbox.nearest(event.y)
            if idx is None:
                return
            # ensure the click was actually within the item's bbox (not below the last item)
            bbox = self.listbox.bbox(idx)
            if not bbox:
                return
            x, y, w, h = bbox
            if event.y < y or event.y > (y + h):
                return
            # programmatically set selection to this index
            self.listbox.selection_clear(0, 'end')
            self.listbox.selection_set(idx)
            self.listbox.activate(idx)
            self.selected_robot_index = idx
            # open editor for the selected robot
            self.edit_selected()
        except Exception:
            # on any error, don't crash the UI
            return



    def build_cfg_dict(self):
        """Build the fleet configuration dictionary."""
        world = self.world_cb.get().strip()
        if not world:
            messagebox.showerror("Error", "Please select a world file")
            return None
        
        if not self.robots:
            messagebox.showerror("Error", "Please add at least one robot to the fleet")
            return None
        
        # Extract just the filename for worlds in the package worlds directory
        world_filename = os.path.basename(world)
        
        # Extract world name without extension
        world_name = os.path.splitext(world_filename)[0]
        
        cfg = {
            'world_file': world_filename,  # Just the filename - launch file handles the path
            'world_name': world_name,
            'robot_ns': self.ns_entry.get().strip() or "robot",
            'robots': self.robots
        }
        
        # Validate configuration
        try:
            validate_config(cfg)
        except ValueError as e:
            messagebox.showerror("Configuration Error", str(e))
            return None
        
        return cfg

    def save_config(self):
        """Save the fleet configuration to a YAML file."""
        cfg = self.build_cfg_dict()
        if cfg is None:
            return
        
        # Default target directory
        target = DEFAULT_CONFIG_DIR if os.path.isdir(DEFAULT_CONFIG_DIR) else os.getcwd()
        default_filename = "fleet_config.yaml"
        
        # Ask user for save location
        out = filedialog.asksaveasfilename(
            initialfile=default_filename,
            initialdir=target,
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if not out:
            return
        
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(out), exist_ok=True)
            
            # Save using PyYAML
            with open(out, "w") as f:
                yaml.dump(cfg, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
            
            logger.info(f"Configuration saved to {out}")
            messagebox.showinfo("Success", f"Configuration saved to\n{out}")
        except Exception as e:
            logger.error(f"Failed to save configuration: {e}")
            messagebox.showerror("Error", f"Failed to save configuration:\n{e}")

    def save_and_launch(self):
        """Save the configuration and launch the simulator."""
        cfg = self.build_cfg_dict()
        if cfg is None:
            return
        
        # Check if a process is already running
        if self.launched_process and self.launched_process.poll() is None:
            if not messagebox.askyesno("Confirm", "A simulator is already running. Terminate it and launch a new one?"):
                return
            self._terminate_processes()
        
        # Determine save location
        if os.path.isdir(DEFAULT_CONFIG_DIR):
            out = os.path.join(DEFAULT_CONFIG_DIR, "fleet_config.yaml")
        else:
            out = filedialog.asksaveasfilename(
                initialfile="fleet_config.yaml",
                defaultextension=".yaml",
                filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")]
            )
            if not out:
                return
        
        try:
            # Save configuration
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "w") as f:
                yaml.dump(cfg, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
            
            logger.info(f"Configuration saved to {out}")
            
            # Launch the simulator
            messagebox.showinfo("Launching", f"Configuration saved to:\n{out}\n\nLaunching simulator...")
            cmd = ["ros2", "launch", "multirobot_gz_simulator", "fleet.launch.py"]
            logger.info(f"Launching: {' '.join(cmd)}")
            # Start the process in its own process group so we can kill the whole group later
            self.launched_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            logger.info(f"Process launched with PID {self.launched_process.pid}")
            
        except FileNotFoundError:
            logger.error("ros2 command not found. Make sure ROS 2 is installed and sourced.")
            messagebox.showerror("Launch Failed", "ros2 command not found.\nMake sure ROS 2 is installed and sourced.")
        except Exception as e:
            logger.error(f"Launch failed: {e}")
            messagebox.showerror("Launch Failed", f"Failed to launch simulator:\n{e}")

if __name__ == "__main__":
    try:
        app = FleetGUI()
        app.mainloop()
    except Exception as e:
        logger.error(f"Application error: {e}", exc_info=True)
        import traceback
        traceback.print_exc()