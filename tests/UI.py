import PySimpleGUI as sg

sg.theme('DarkAmber')  # Add a touch of color
# All the stuff inside your window.
layout = [
    [sg.Text('Some text on Row 1')],
    [sg.Text('Some text on Row 1'), sg.Slider(range=(0, 500), default_value=222, size=(20, 15), orientation='horizontal', font=('Helvetica', 12), key="slider")]
]

# Create the Window
window = sg.Window('Window Title', layout)
# Event Loop to process "events" and get the "values" of the inputs
while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Cancel':  # if user closes window or clicks cancel
        break
    print('You entered ', values[0])

window.close()
