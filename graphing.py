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

# new0 = list()
# new12 = list()
# newX = list()
# for num in rotated[0]:
#     new0.append((num-12)/10)
# numsSinceLastDrop = 0
# for index in range(rotated[12]):
#     if index >= 0:
#         if(rotated[12][index]-rotated[12][index-1]<0):
#             new12.append(numsSinceLastDrop)
#             numsSinceLastDrop = 0
#         else:
#             numsSinceLastDrop += 1

# plot = px.scatter(x=rotated[0], y=[rotated[12],rotated[12],rotated[3],rotated[4],rotated[7],rotated[8],rotated[9],rotated[10],rotated[11],rotated[12],rotated[13]])

# plot = px.scatter(x=rotated[0],y=[rotated[0], rotated[13]])
# newnames = {
#     'wide_variable_0:Velocity',
#     'wide_variable_1:Filtered Velocity',
#     'wide_variable_2:Filtered Error',
#     'wide_variable_3:Velocity Target'
# }
# plot.for_each_trace(lambda t: t.update(name = newnames[t.name]))

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
differenceList = list()
new16 = list()
for i in rotated[16]:
    new16.append(i)
new0 = list()
for i in rotated[0]:
    new0.append(i)
for i in range(len(new16)):
    differenceList.append(new16[i]-new0[i])

plot = px.scatter(x=rotated[0], y=[differenceList])
plot.show()

