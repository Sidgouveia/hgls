from vrp import cvrp
from os import listdir
from multiprocessing import Process
import json
import pandas

def save(cvrp, run):
    file = open('solutions/files/'+cvrp.s['instance']+'_'+str(run+1)+'.sol', 'w')
    for k, route in enumerate(cvrp.s['routes']):
        line = ''
        for node in route:
            line += ' {}'.format(cvrp.original_locations.index(cvrp.locations[node]))
        file.write('Route #'+str(k+1)+': '+line+'\n')
    file.write('Cost '+str(cvrp.s['cost']))
    file.close
    pass

def export_json(S, instance):
    file = open('C:/Users/gouve/Desktop/Trabalho Final/VRP/solutions/json/'+instance+'.json', 'w')
    file.write(json.dumps(S))
    file.close()
    pass

def export_csv(S, instance):
    pandas.DataFrame([[S[i]['instance'], S[i]['num_vehicles'], S[i]['cost'], S[i]['gap']] for i in S], columns=['instance', 'num_vehicles', 'cost', 'gap']).to_csv('solutions/csv/'+instance+'.csv', index=False)
    pass

def main():
    for instance in listdir('instances'):
        S = {}
        vrp = cvrp()
        vrp.read(instance[:-4])
        for i in range(10):
            process = Process(target=vrp.guided_local_search)
            process.run()
            S[i] = vrp.s
            print('instance: '+'{:12}'.format(instance[:-4]), 'id: '+'{:2}'.format(str(i)), 'cost: '+'{:8}'.format(str(vrp.s['cost'])), 'gap: '+'{:6}'.format(str(round(vrp.s['gap'], 2))))
            save(vrp, i)
            process.close()
            del process
        del vrp
        export_json(S, instance[:-4])
        export_csv(S, instance[:-4])
if __name__ == '__main__':
    main()