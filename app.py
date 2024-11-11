import tkinter as tk
from tkinter import filedialog, scrolledtext
from PIL import Image, ImageTk
from extract import getDir
from load_map import loadMap
from collections import deque
from algorithms import estimate_max_capacity
from ford_fulkerson import fordFulkerson
from edmonds_karp import find_maximum_flow_using_edmonds_karp, find_maximum_flow_using_edmonds_karp_multidigraph
from dinics import dinics
import time

ratio = 111196.2878

class ImageCanvasApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Image Canvas")
        
        self.image = None
        self.image_layer = None
        self.node_layer = []
        self.edge_layer = []
        
        self.map_name = 'map_all.osm'
        self.graph = loadMap(self.map_name)
        # create capacity for edges
        for node in self.graph.nodes(data=True):
            node[1]['level'] = -1
            #print(node)
        for edge in self.graph.edges(data=True):
            #print(f'old edge: {edge}')
            edge[2]['flow'] = 0
            edge[2]['capacity'] = estimate_max_capacity(edge[2])
            #print(f'new edge: {edge}')
            
        print(self.graph)
        self.bbox = {
            'max_lon': 106.71553,
            'min_lon': 106.65416,
            'max_lat': 10.81754,
            'min_lat': 10.76729,
            'range_lon': 106.71553 - 106.65416,
            'range_lat': 10.81754 - 10.76729
            }
        self.dim = {'width': 1065, 'height': 728}
        self.ratio = {'x': self.dim['width']/self.bbox['range_lon'], 'y': self.dim['height']/self.bbox['range_lat']}
        self.dist = 3
        self.start_node = None
        self.end_node = None

        # Create an entry widget for user input
        #self.entry = tk.Entry(self.root, width=40)
        #self.entry.pack(side=tk.LEFT, padx=10, pady=10)  # Pack the entry on the left

        self.left_frame = tk.Frame(self.root)
        self.left_frame.pack(side=tk.LEFT, padx=0, pady=0)

        # Create a button that will call the print_text function when clicked
        self.button_1 = tk.Button(self.left_frame, text="Toggle nodes", command=self.toggle_node)
        self.button_1.pack(pady=0)
        
        # ford fulkerson
        self.button_2 = tk.Button(self.left_frame, text="forful", command=self.ff_flow_paths)
        self.button_2.pack(pady=0)

        # edmonds karp
        self.button_3 = tk.Button(self.left_frame, text="edkarp", command=self.ek_flow_paths)
        self.button_3.pack(pady=0)

        # dinics
        self.button_4 = tk.Button(self.left_frame, text="dinics", command=self.dinics_flow_paths)
        self.button_4.pack(pady=0)

        # edmonds karp shortest path
        self.button_5 = tk.Button(self.left_frame, text="edkarp_shortest_path", command=self.ek_shortest_paths)
        self.button_5.pack(pady=0)

        # dinics shortest path
        self.button_6 = tk.Button(self.left_frame, text="dinics_shortest_path", command=self.dinics_shortest_paths)
        self.button_6.pack(pady=0)

        # Create a ScrolledText widget for the print area
        self.print_area = scrolledtext.ScrolledText(self.left_frame, width=50, height=40, wrap=tk.WORD, font = ("Calibri", 8))
        self.print_area.pack(padx=5, pady=0)
        
        # pack canvas after all buttons
        self.canvas = tk.Canvas(self.root, width=self.dim['width'], height=self.dim['height'], bg='white')
        self.canvas.pack(side=tk.LEFT, padx=5, pady=10)
        
        # Load an image
        self.load_image(getDir('map_new.png'))
        
        self.canvas.bind("<Button-1>", self.on_mouse_lclick)
        self.canvas.bind("<Button-2>", self.on_mouse_rclick)
        self.canvas.bind("<Button-3>", self.on_mouse_rclick)
        self.draw_map()
        
        self.test()
        return

    def reorient(self, x, y):
        # This function inverts the Y coordinate to simulate a bottom-left origin
        return x, self.dim['height'] - y
    
    def load_image(self, img_file):
        self.image = Image.open(img_file)
        #self.image.thumbnail((800, 600))  # Resize image to fit the canvas
        self.tk_image = ImageTk.PhotoImage(self.image)
        if self.image_layer is not None:
            self.canvas.delete(self.image_layer)  # Remove previous image if any
        
        self.image_layer = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        return
    
    def log_message(self, message):
        self.print_area.insert(tk.END, message + "\n")
        self.print_area.yview(tk.END)  # Auto-scroll to the bottom

    
    def convert_coord_2_pixl(self, lon, lat):
        x = (lon - self.bbox['min_lon']) * self.ratio['x']
        y = (lat - self.bbox['min_lat']) * self.ratio['y']
        return x, y
    
    def convert_pixl_2_Coord(self, x, y):
        lon = x / self.ratio['x'] + self.bbox['min_lon']
        lat = y / self.ratio['y'] + self.bbox['min_lat']
        return lon, lat
    
    def draw_map(self):
        for node in self.graph.nodes(data=True):
            if 'x' in node[1]:
                #print(node[1]['lon'])
                x, y = self.convert_coord_2_pixl(node[1]['x'], node[1]['y'])
                #print(x, y)
                x, y = self.reorient(x, y)
                #x, y = int(x), int(y)
                #print(x, y)
                rect = self.canvas.create_rectangle(x-self.dist, y-self.dist, x+self.dist, y+self.dist, outline='red')
                self.node_layer.append([node[0], rect])
        return
    
    def on_mouse_lclick(self, event, color='blue'):
        x, y = event.x, event.y
        x, y = self.reorient(x, y)
        lon, lat = self.convert_pixl_2_Coord(x, y)
        if self.start_node:
            self.canvas.itemconfig(self.start_node[1], outline="red")
        self.start_node = self.find_closest_point(lon, lat, color=color)
        if self.start_node:
            self.canvas.itemconfig(self.start_node[1], outline="blue")
            self.log_message(f"Left clicked at: <<{self.start_node[0]}>> ({x}, {y}) => ({lon}, {lat}) => node: {self.start_node}")
        return self.start_node
    
    def on_mouse_rclick(self, event, color='green'):
        x, y = event.x, event.y
        x, y = self.reorient(x, y)
        lon, lat = self.convert_pixl_2_Coord(x, y)
        if self.end_node:
            self.canvas.itemconfig(self.end_node[1], outline="red")
        self.end_node = self.find_closest_point(lon, lat, color=color)
        if self.end_node:
            self.canvas.itemconfig(self.end_node[1], outline="green")
            self.log_message(f"Right clicked at: <<{self.end_node[0]}>> ({x}, {y}) => ({lon}, {lat}) => node: {self.end_node}")
        return self.end_node
        
    def find_closest_point(self, lon, lat, color='red'):
        for node_rect in self.node_layer:
            node_id = node_rect[0]
            node = self.graph.nodes[node_id]
            if 'x' not in node:
                continue
            if abs(node['x'] - lon) < self.dist/10000 and abs(node['y'] - lat) < self.dist/10000:
                return node_rect
        return None
    
    def ff_flow_paths(self):
        if not self.start_node:
            self.log_message('No start node picked')
            return
        if not self.end_node:
            self.log_message('No end node picked')
            return
        
        start_time = time.time()
        max_flow, paths = fordFulkerson(self.graph, self.start_node[0], self.end_node[0])
        end_time = time.time()
        runtime = end_time - start_time
                
        for path in paths:
            self.log_message(f'path: {path}')
        self.log_message(f'Found: {len(paths)} paths')
        self.log_message(f'Max flow: {max_flow}')
        self.log_message(f'Runtime: {runtime}')
        return
    
    def ek_flow_paths(self):
        if not self.start_node:
            self.log_message('No start node picked')
            return
        if not self.end_node:
            self.log_message('No end node picked')
            return
        
        start_time = time.time()
        max_flow, paths = find_maximum_flow_using_edmonds_karp_multidigraph(self.graph, self.start_node[0], self.end_node[0])
        end_time = time.time()
        runtime = end_time - start_time
                
        for path in paths:
            self.log_message(f'path: {path}')
        self.log_message(f'Found: {len(paths)} paths')
        self.log_message(f'Max flow: {max_flow}')
        self.log_message(f'Runtime: {runtime}')
        return
    
    def dinics_flow_paths(self):
        if not self.start_node:
            self.log_message('No start node picked')
            return
        if not self.end_node:
            self.log_message('No end node picked')
            return
        
        start_time = time.time()
        max_flow, paths = dinics(self.graph, self.start_node[0], self.end_node[0], shortest_dist=None)
        end_time = time.time()
        runtime = end_time - start_time
                
        for path in paths:
            self.log_message(f'path: {path}')
        self.log_message(f'Found: {len(paths)} paths')
        self.log_message(f'Max flow: {max_flow}')
        self.log_message(f'Runtime: {runtime}')
        return
    
    def ek_shortest_paths(self):
        if not self.start_node:
            self.log_message('No start node picked')
            return
        if not self.end_node:
            self.log_message('No end node picked')
            return
        
        start_time = time.time()
        max_flow, paths = find_maximum_flow_using_edmonds_karp_multidigraph(self.graph, self.start_node[0], self.end_node[0], shortest_dist='cond_1')
        end_time = time.time()
        runtime = end_time - start_time
                
        for path in paths:
            self.log_message(f'path: {path}')
        self.log_message(f'Found: {len(paths)} paths')
        self.log_message(f'Max flow: {max_flow}')
        self.log_message(f'Runtime: {runtime}')
        return
    
    def dinics_shortest_paths(self):
        if not self.start_node:
            self.log_message('No start node picked')
            return
        if not self.end_node:
            self.log_message('No end node picked')
            return
        
        start_time = time.time()
        max_flow, paths = dinics(self.graph, self.start_node[0], self.end_node[0], shortest_dist='cond_1')
        end_time = time.time()
        runtime = end_time - start_time
                
        for path in paths:
            self.log_message(f'path: {path}')
        self.log_message(f'Found: {len(paths)} paths')
        self.log_message(f'Max flow: {max_flow}')
        self.log_message(f'Runtime: {runtime}')
        return
    
    def toggle_node(self):
        # Toggle the visibility of the drawing layer
        if self.node_layer:
            for node in self.node_layer:
                current_state = self.canvas.itemcget(node[1], "state")
                new_state = "hidden" if current_state == "normal" else "normal"
                self.canvas.itemconfigure(node[1], state=new_state)

    def test(self):
        #for node in self.graph.nodes:
        #    print(f'node: {self.graph.nodes[node]}')
        #    print(f'edge: {self.graph.edges(node, data=True)}')
        #    print(f'edge i: {self.graph.in_edges(node)}')
        #    print(f'edge o: {self.graph.out_edges(node)}')
        return

if __name__ == "__main__":
    root = tk.Tk()
    app = ImageCanvasApp(root)
    root.mainloop()
