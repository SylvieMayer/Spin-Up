import plotly.express as px
import csv
import datetime
import numpy as np
import pandas
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import time
from scipy import optimize

# external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
# external_stylesheets = ['./styles.css']
app = dash.Dash(__name__) #
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
    # # s_t = (time.time()-global_start)*1000
    # data = list()
    # with open("data.csv") as f:
    #     csv_data = csv.reader((line.replace('\0','') for line in f), delimiter="|")
    #     for row in csv_data:
    #         if(len(row) > 0):
    #             try:
    #                 for index, field in enumerate(row):
    #                     row[index] = int(float(field))
    #                 data.append(row)
    #             except:
    #                 continue
    # rotated = np.transpose(data)
    # plot = px.scatter(x=rotated[0], y=[rotated[1], rotated[3], rotated[7]], height=800)
    # return plot
    
    df = pandas.read_csv('data.csv',delimiter=',',names =['time (ms)','frisbees in robo','theta target'],
                        skiprows=50,encoding='utf-16',skipfooter=50,engine='python')
    plot = px.scatter(df,x='time (ms)',y=['frisbees in robo'],height=800)
    return plot

    
    

    

if __name__ == '__main__':
    app.run_server(debug=True)


# data = list()
# with open("data.csv") as f:
#     csv_data = csv.reader((line.replace('\0','') for line in f), delimiter="|")
#     for row in csv_data:
#         if(len(row) > 0):
#             try:
#                 for index, field in enumerate(row):
#                     row[index] = int(float(field))
#                 data.append(row)
#             except:
#                 continue
# rotated = numpy.transpose(data)

# plot = px.scatter(x=rotated[0], y=[rotated[1],rotated[2],rotated[3],rotated[4], rotated[5], rotated[6],rotated[7]])
# # plot = px.scatter(x=rotated[0], y=[rotated[1],rotated[2], rotated[3]])
# plot.show()

