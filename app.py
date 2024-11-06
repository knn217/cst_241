import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
from extract import getDir
from load_map import loadMap

ratio = 111196.2878

class ImageCanvasApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Image Canvas")
        
        self.dim = {'width': 1344, 'height': 686}
        self.canvas = tk.Canvas(root, width=self.dim['width'], height=self.dim['height'], bg='white')
        self.canvas.pack()

        self.image = None
        self.image_id = None
        
        self.map_name = 'newgraph_conso.osm'
        self.graph = loadMap(self.map_name)
        self.bbox = {
            'max_lon': 106.71535,
            'min_lon': 106.64738,
            'max_lat': 10.81864,
            'min_lat': 10.78786,
            'range_lon': 106.71535 - 106.64738,
            'range_lat': 10.81864 - 10.78786
            }
        self.ratio = {'x': self.dim['width']/self.bbox['range_lon'], 'y': self.dim['height']/self.bbox['range_lat']}
        self.dist = 3
        self.start_node = None
        self.end_node = None

        # Load an image
        self.load_image(getDir('map.png'))
        self.load_image_button = tk.Button(root, text="Reset", command=self.draw_map)
        self.load_image_button.pack()
        
        # Create an entry widget for user input
        self.entry = tk.Entry(self.root, width=40)
        self.entry.pack(side=tk.LEFT, padx=10, pady=10)  # Pack the entry on the left

        # Create a button that will call the print_text function when clicked
        self.print_button = tk.Button(self.root, text="Print Text", command=self.save_as)
        self.print_button.pack(side=tk.LEFT, padx=10, pady=10)  # Pack the button next to the entry

        self.canvas.bind("<Button-1>", self.on_mouse_lclick)
        self.canvas.bind("<Button-2>", self.on_mouse_rclick)
        self.canvas.bind("<Button-3>", self.on_mouse_rclick)
        self.draw_map()

    def load_image(self, img_file):
        self.image = Image.open(img_file)
        #self.image.thumbnail((800, 600))  # Resize image to fit the canvas
        self.tk_image = ImageTk.PhotoImage(self.image)
        if self.image_id is not None:
            self.canvas.delete(self.image_id)  # Remove previous image if any
        
        self.image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        return
    
    def convert_coord_2_pixl(self, lon, lat):
        x = (lon - self.bbox['min_lon']) * self.ratio['x']
        y = (lat - self.bbox['min_lat']) * self.ratio['y']
        return x, y
    
    def convert_pixl_2_Coord(self, x, y):
        lon = x / self.ratio['x'] + self.bbox['min_lon']
        lat = y / self.ratio['y'] + self.bbox['min_lat']
        return lon, lat
    
    def draw_map(self):
        #print(len(self.graph._node))
        #print(len(self.graph.edges))
        #self.canvas.create_rectangle(357, 171, 357, 171, outline='red')
        for node in self.graph.nodes(data=True):
            if 'lon' in node[1]:
                #print(node[1]['lon'])
                x, y = self.convert_coord_2_pixl(node[1]['lon'], node[1]['lat'])
                #x, y = int(x), int(y)
                #print(x, y)
                self.canvas.create_rectangle(x-self.dist, y-self.dist, x+self.dist, y+self.dist, outline='red')
            #else:
            #    #print(node[1])
            #    pass
        #for edge in self.graph.edges(data=True):
        #    print(edge)
        return
    
    def on_mouse_lclick(self, event, color='blue'):
        x, y = event.x, event.y
        lon, lat = self.convert_pixl_2_Coord(x, y)
        self.start_node = self.mark_closest_point(lon, lat, color=color)
        print(f"Clicked at: ({x}, {y}) => ({lon}, {lat}) => node: {self.start_node}")
        return self.start_node
    
    def on_mouse_rclick(self, event, color='green'):
        x, y = event.x, event.y
        lon, lat = self.convert_pixl_2_Coord(x, y)
        self.end_node = self.mark_closest_point(lon, lat, color=color)
        print(f"Clicked at: ({x}, {y}) => ({lon}, {lat}) => node: {self.end_node}")
        return self.end_node
        
    def save_as(self):
        file_name = self.entry.get()  # Get text from the entry widget
        print(file_name)  # Print the text to the terminal
        return
    
    def mark_closest_point(self, lon, lat, color='red'):
        dist_nodes = []
        for node in self.graph.nodes(data=True):
            if 'lon' in node[1]:
                if abs(node[1]['lon'] - lon) < self.dist/10000 and abs(node[1]['lat'] - lat) < self.dist/10000:
                    x, y = self.convert_coord_2_pixl(node[1]['lon'], node[1]['lat'])
                    self.canvas.create_rectangle(x-self.dist, y-self.dist, x+self.dist, y+self.dist, outline=color)
                    #dist_nodes.append(node)
                    return node        
        return None


if __name__ == "__main__":
    root = tk.Tk()
    app = ImageCanvasApp(root)
    root.mainloop()
