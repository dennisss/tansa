#JOCS: JOCS Open Choreography Schema
Examples as well as documentation of what's going on in each line are listed below

### Six Drones Moving in Two Groups
```javascript
// JOCS: JOCS Open Choreography Schema
{
  // Stores starting data for each drone present
  "drones": [ 
    {
      // startPosition is their "home"
      // This is the point (midair) where the choreography starts, AFTER takeoff happens
      "startPosition" : [0, -5, 1], 
      
      // id referes to a unique identifier for each drone.
      // This is redundant if drones are stored in an array in code and the id matches the index.
      "id": 0 
    },
    {
      "startPosition" : [0, -3, 1],
      "id": 1
    },
    {
      "startPosition" : [0, -1, 1],
      "id": 2
    },
    {
      "startPosition" : [0, 1, 1],
      "id": 3
    },
    {
      "startPosition" : [0, 3, 1],
      "id": 4
    },
    {
      "startPosition" : [0, 5, 1],
      "id": 5
    }
  ],
  
  // Holds a list of start time groups.
  // Each group holds choreography items starting at each given time.
  "chor": [ 
    {
      // Start time for this group of actions, in s
      "time": 0.0,
      
      // List of actions to be sent to drones at this start time
      // This specific action tells half the drones to go one way
      // and the other half to go another way, in relation to their "home"s
      "action": [ 
        {
          // This string should not have a typo and should match a supported trajectory type
          "type": "line",
          
          // Values in this will differ based on action type
          "data":
          {
            "startPointCenter": [0, 0, 0], // Offset is in respect to each drone's "home" location defined above
            "endPointCenter": [-3, 0, 0] // This (and the line above) will be ADDED to the "home" location of each respective drone
          },
          
          // Duration of this trajectory in s
          "duration": 5.00,
          
          // Should include 1+ drone numbers, these are the drones this trajectory applies to
          "drones": [0,2,4]
        },
        {
          // This is just another trajectory that should happen at the same time as the previous
          "type": "line",
          "data":
          {
            "startPointCenter": [0, 0, 0],
            "endPointCenter": [3, 0, 0]
          },
          "duration": 5.00,
          "drones": [1,3,5]
        }
      ]
    }
  ]
}
```


###One drone moving in a circle then square
Documentation regarding things that were already explained above were left out in this documentation
```javascript
{
  "drones": [
    {
      "startPosition" : [0, 0, 1],
      "id": 0
    }
  ],
  "chor": [
    {
      "time": 0.0,
      "action": [
        {
          // A "transition" action does not have any "data" specifications.
          // In code, we gather how long a transition should take, and calculate
          // the relevant data to actually perform the transition.
          "type": "transition", 

          // However, "data" should not be null, just empty.
          "data": {},

          "duration": 5.0,
          "drones": [0]
        }
      ]
    },
    {
      "time": 5.0,
      "action": [
        {
          "type": "circle",
          "data":
          {
            // origin represents the center of the circle
            "originPointCenter": [0,0,1],
            // radius is in stored "meters", and will be read as a double in the code
            "radius": 2,
            // theta1 represents the spot on the circle to begin, in degrees
            "theta1": 0,
            // theta2 represents the spot on the circle to cease this trajectory on, in degrees
            "theta2": 360
          },
          "duration": 15,
          "drones": [0]
        }
      ]
    },
    {
      "time": 20.0,
      "action": [
        {
          "type": "transition",
          "data": {},
          "duration": 2.5,
          "drones": [0]
        }
      ]
    },
    {
      "time": 22.5,
      "action": [
        {
          "type": "line",
          "data":
          {
            // In a line, unique data needed are the start point and end point
            "startPointCenter": [2, 2, 1],
            "endPointCenter": [2, -2, 1]
          },
          "duration": 5,
          "drones": [0]
        }
      ]
    },
    {
      "time": 27.5,
      "action": [
        {
          "type": "line",
          "data":
          {
            "startPointCenter": [2, -2, 1],
            "endPointCenter": [-2, -2, 1]
          },
          "duration": 5,
          "drones": [0]
        }
      ]
    },
    {
      "time": 32.5,
      "action": [
        {
          "type": "line",
          "data":
          {
            "startPointCenter": [-2, -2, 1],
            "endPointCenter": [-2, 2, 1]
          },
          "duration": 5,
          "drones": [0]
        }
      ]
    },
    {
      "time": 37.5,
      "action": [
        {
          "type": "line",
          "data":
          {
            "startPointCenter": [-2, 2, 1],
            "endPointCenter": [0, 0, 1]
          },
          "duration": 5,
          "drones": [0]
        }
      ]
    }
  ]
}
```
