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
        
        self.dim = {'width': 795, 'height': 366}
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

        # Load an image
        self.load_image_button = tk.Button(root, text="Switch View", command=self.load_image)
        self.load_image(getDir('./map.png'))
        self.load_image_button.pack()
        
        # Create an entry widget for user input
        self.entry = tk.Entry(self.root, width=40)
        self.entry.pack(side=tk.LEFT, padx=10, pady=10)  # Pack the entry on the left

        # Create a button that will call the print_text function when clicked
        self.print_button = tk.Button(self.root, text="Print Text", command=self.save_as)
        self.print_button.pack(side=tk.LEFT, padx=10, pady=10)  # Pack the button next to the entry

        self.canvas.bind("<Button-1>", self.get_click_position)
        self.draw_map()

    def load_image(self, img_file):
        self.image = Image.open(img_file)
        #self.image.thumbnail((800, 600))  # Resize image to fit the canvas
        self.tk_image = ImageTk.PhotoImage(self.image)
        if self.image_id is not None:
            self.canvas.delete(self.image_id)  # Remove previous image if any
        
        self.image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        return
    
    def convertCoord2Pixl(self, lon, lat):
        x = (lon - self.bbox['min_lon']) * self.ratio['x']
        y = (lat - self.bbox['min_lat']) * self.ratio['y']
        return x, y
    
    def convertPixl2Coord(self, x, y):
        lon = x / self.ratio['x'] + self.bbox['min_lon']
        lat = y / self.ratio['y'] + self.bbox['min_lat']
        return lon, lat
    
    def draw_map(self):
        print(len(self.graph._node))
        print(len(self.graph.edges))
        for node in self.graph.nodes(data=True):
            #print(node[1])
            #x, y = self.convertCoord2Pixl(node)
            self.canvas.create_rectangle(50, 0, 100, 50, outline='red')
        return
    
    def get_click_position(self, event):
        x, y = event.x, event.y
        lon, lat = self.convertPixl2Coord(x, y)
        print(f"Clicked at: ({x}, {y}) => ({lon}, {lat})")
        return
        
    def save_as(self):
        file_name = self.entry.get()  # Get text from the entry widget
        print(file_name)  # Print the text to the terminal
        return


if __name__ == "__main__":
    root = tk.Tk()
    app = ImageCanvasApp(root)
    root.mainloop()
