import plotly.express as px
import csv
import datetime
import numpy
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import time

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)
app.layout = html.Div(
    html.Div([
        html.H4('V5 Info'),
        html.Div(id='live-update-text'),
        dcc.Graph(id='live-update-graph'),
        dcc.Interval(
            id='interval-component',
            interval=1*300, # in milliseconds
            n_intervals=0
        )
    ])
)
# global_start = time.time()
@app.callback(Output('live-update-graph', 'figure'),
              Input('interval-component', 'n_intervals'))
def update_graph_live(n):
    # s_t = (time.time()-global_start)*1000
    data = list()
    with open("data.csv") as f:
        csv_data = csv.reader((line.replace('\0','') for line in f), delimiter="|")
        for row in csv_data:
            if(len(row) > 0):
                try:
                    for index, field in enumerate(row):
                        row[index] = int(float(field))
                    data.append(row)
                except:
                    continue
    rotated = numpy.transpose(data)
    # reality_drift = s_t-rotated[0][-1]
    # print(reality_drift)
    plot = px.scatter(x=rotated[0], y=[rotated[0],rotated[11], rotated[16]])
    

    return plot

# if __name__ == '__main__':
#     app.run_server(debug=True)


data = list()
with open("data.csv") as f:
    csv_data = csv.reader((line.replace('\0','') for line in f), delimiter="|")
    for row in csv_data:
        if(len(row) > 0):
            try:
                for index, field in enumerate(row):
                    row[index] = int(float(field))
                data.append(row)
            except:
                continue
rotated = numpy.transpose(data)

plot = px.scatter(x=rotated[0], y=[rotated[1],rotated[2],rotated[4],rotated[5],rotated[6],rotated[7],rotated[8]])
plot.show()

