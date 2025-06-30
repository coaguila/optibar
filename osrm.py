import requests

## OSRM API Endpoint
OSRM = "http://router.project-osrm.org/route/v1/foot"

def get_route (a, b):
    ## build route
    url = OSRM + a + ";" + b

    ## response
    res = requests.get(url)

    return res.json()