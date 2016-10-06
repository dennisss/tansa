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
            "startPointOffset": [0, 0, 0], // Offset is in respect to each drone's "home" location defined above
            "endPointOffset": [-3, 0, 0] // This (and the line above) will be ADDED to the "home" location of each respective drone
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
            "startPointOffset": [0, 0, 0],
            "endPointOffset": [3, 0, 0]
          },
          "duration": 5.00,
          "drones": [1,3,5]
        }
      ]
    }
  ]
}
```
