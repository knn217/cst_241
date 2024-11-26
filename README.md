## Graph Assignment 
## Teacher: TS Tran Anh Tuan
## cst_241

### NOTE: 
> The instructions below is for the steps to setup and run the app from source code
> However, some of our team members have run into error with tkinter after creating a virtual environment
> So we have also prepared a built app for Windows to run without setting up

### STEPS: 
Step 1: Clone this repository or download and unzip the file "cst_241-main.zip"

Step 2: Navigate to this project's directory:
```bash
cd path\to\project\directory
```
Example:
```bash
cd C:\Users\Downloads\cst_241-main\cst_241-main
```

Step 3: Create a new virtual environment:
```bash
python -m venv venv
```

Step 4: Activate the virtual environment:
- For Windows:
```bash
.\venv\Scripts\activate
```
- For MacOS/Linux:
```bash
source venv/bin/activate
```

Step 5: Install dependencies:
```bash
pip install -r requirements.txt
```

Step 6: Run app.py
```bash
python app.py
```