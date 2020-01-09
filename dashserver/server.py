#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.2
# DASH-DAQ Server: A web server for the York Robotics Kit
# James Hilder, York Robotics Laboratory, Jan 2020

"""
.. module:: server
   :synopsis: A DASH-DAQ web server for debugging and controlling YRK robots

.. moduleauthor:: James Hilder <github.com/jah128>



To run::

   python server.py

"""

import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import dash_daq as daq
import pickle

from dashserver.app import app
import yrk.settings as settings
from dashserver.tabs import camera_tab
#from tabs import system_tab, control_tab, camera_tab, analog_tab #, robot_tab

app.index_string = '''
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>York Robotics Kit</title>
        {%favicon%}
        {%css%}
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
        <div>(C) James Hilder, York Robotics Laboratory, University of York, Jan 2020</div>
    </body>
</html>
'''

tabs_styles = {
    'height': '36px'
}

tab_style = {
    'borderBottom': '1px solid #d6d6d6',
    'padding': '6px',
    'fontWeight': 'bold'
}

tab_selected_style = {
    'borderTop': '1px solid #d6d6d6',
    'borderBottom': '1px solid #d6d6d6',
    'backgroundColor': '#119DFF',
    'color': 'white',
    'padding': '6px'
}

root_layout = html.Div(
    [
        dcc.Location(id="url", refresh=False),
        html.Div(
            [
                daq.ToggleSwitch(
                    id="toggleTheme",
                    style={"position": "absolute", "transform": "translate(-50%, 20%)"},
                    size=25,
                )
            ],
            id="toggleDiv",
            style={"width": "fit-content", "margin": "0 auto"},
        ),
        html.Div(id="page-content"),
    ]
)

app.layout = root_layout

tab_list = []
if(settings.ENABLE_ROBOT_TAB): tab_list = [dcc.Tab(label='Robot', value='robot-tab', style=tab_style, selected_style=tab_selected_style)]
tab_list.extend([
    dcc.Tab(label='System', value='system-tab', style=tab_style, selected_style=tab_selected_style),
    dcc.Tab(label='Control', value='control-tab', style=tab_style, selected_style=tab_selected_style),
    dcc.Tab(label='Sensors', value='analog-tab', style=tab_style, selected_style=tab_selected_style),
    dcc.Tab(label='Camera', value='camera-tab', style=tab_style, selected_style=tab_selected_style),
    dcc.Tab(label='Documents', value='documents-tab', style=tab_style, selected_style=tab_selected_style)
])

app.layout = html.Div(
    [
        html.Div(
            id="container",
            style={"background-color": "#e26618"},
            children=[
                html.H2("York Robotics Kit"),
                html.A(
                    html.Img(src='assets/uoy-logo.png')
                ),
            ],
            className="banner",
        ),
        dcc.Tabs(id="tabs", value='tab-1-example', children=tab_list, style=tabs_styles),
        dcc.Store(id='memory'),
        html.Div(id='tabs-pages')
    ],
    style={
        "padding": "0px 10px 10px 10px",
        "marginLeft": "auto",
        "marginRight": "auto",
        "width": "960",
        "height": "1800",
        "boxShadow": "0px 0px 5px 5px rgba(204,204,204,0.4)",
    },
)

app.scripts.config.serve_locally = True
app.config["suppress_callback_exceptions"] = True


@app.callback(Output('tabs-pages', 'children'),
              [Input('tabs', 'value')])
def render_content(tab):
    if tab == 'camera-tab': return camera_tab.layout
    elif tab == 'documents-tab':
        return html.Iframe(src='html/index.html')
    return ""

    # if tab == 'system-tab': return system_tab.layout
    # elif tab == 'control-tab': return control_tab.layout
    # elif tab == 'camera-tab': return camera_tab.layout
    # elif tab == 'analog-tab': return analog_tab.layout
    # elif tab == 'robot-tab': return robot_tab.layout

def index_run():
    app.run_server(host='0.0.0.0',debug=True,port=8080)

if __name__ == '__main__':
    index_run()
