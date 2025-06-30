import requests

## OSRM API Endpoint
OSRM = "http://router.project-osrm.org/route/v1/foot/"

def get_route_distance(a, b):
    ## build route url
    url = OSRM + a + ";" + b

    ## response
    res = requests.get(url)
    json = res.json()

    return json["routes"][0]["distance"]