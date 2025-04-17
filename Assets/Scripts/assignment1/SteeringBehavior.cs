using UnityEngine;
using System.Collections.Generic;
using TMPro;

public class SteeringBehavior : MonoBehaviour
{
    public Vector3 target;
    public KinematicBehavior kinematic;
    public List<Vector3> path;
    public TextMeshProUGUI label;

    //This is for the waypoint for path-following
    private int currentWaypointIndex = 0;

    //Getting rid of y axis
    Vector3 Flatten(Vector3 v)
    {
        return new Vector3(v.x, 0f, v.z).normalized;
    }

    void Start()
    {
        kinematic = GetComponent<KinematicBehavior>();
        target = transform.position;
        path = null;
        EventBus.OnSetMap += SetMap;
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                SetTarget(hit.point);
            }
        }

        if (path != null && path.Count > 0)
        {
            FollowPath();
        }
        else
        {
            SeekTarget();
        }
    }

    void SeekTarget()
    {
        //setting up initial parameter like position and what position the car is facing
        Vector3 toTarget = target - transform.position;
        float distance = toTarget.magnitude;

        //Distance meter: top left
        if (label != null)
            label.text = $"Dist: {distance:F2}";

        //setting up initial parameter like position and what position the car is facing
        Vector3 forward = transform.forward;
        Vector3 dirToTarget = toTarget.normalized;
        float angle = Vector3.SignedAngle(forward, dirToTarget, Vector3.up);
        float angleAbs = Mathf.Abs(angle);

        // Hardcoded values here
        //(Struggled earlier with the overshooting and the car kept going past the circle and it tries to recorrect itself but never acutally reach the point. one variable has a radius of where I should be stopping and another is the radius of slowing down. )
        float arrivalRadius = 10f;
        float slowingDistance = 20f;

        // Stops when close enough and slow 
        if (distance < arrivalRadius && kinematic.speed < 0.1f)
        {
            kinematic.SetDesiredSpeed(0f);
            kinematic.SetDesiredRotationalVelocity(0f);
            return;
        }

        // Rotation of car
        float rotationT = Mathf.Clamp01(distance / slowingDistance);
        float rotationSpeed = angleAbs < 2f ? 0f :
        Mathf.Clamp(angle * rotationT * 2f, -kinematic.GetMaxRotationalVelocity(), kinematic.GetMaxRotationalVelocity());
        kinematic.SetDesiredRotationalVelocity(rotationSpeed);


        // Speed of Car
        float speedT = Mathf.Clamp01(distance / slowingDistance);
        float speedFactor = Mathf.SmoothStep(0f, 1f, speedT);
        float angleFactor = Mathf.Clamp01((180f - angleAbs) / 180f);
        float desiredSpeed = kinematic.GetMaxSpeed() * speedFactor * angleFactor;

        kinematic.SetDesiredSpeed(desiredSpeed);
    }

    void FollowPath()
    {
        if (path == null || path.Count == 0 || currentWaypointIndex >= path.Count)
        {
            kinematic.SetDesiredSpeed(0f);
            kinematic.SetDesiredRotationalVelocity(0f);
            return;
        }

        //so this part flatten both direction of the xz plane, we will infore the vertical tilt(y-axis)
        Vector3 currentWaypoint = path[currentWaypointIndex];
        Vector3 toWaypoint = currentWaypoint - transform.position;
        float distance = toWaypoint.magnitude;
        Vector3 dirToWaypoint = toWaypoint.normalized;
        Vector3 desiredDirection = dirToWaypoint;
        Vector3 flatForward = Flatten(transform.forward);
        Vector3 flatDesired = Flatten(desiredDirection);

        // Lookahead to see where the next waypoint is
        float advanceRadius = 20f;
        if (currentWaypointIndex + 1 < path.Count)
        {
            Vector3 currentDir = Flatten(dirToWaypoint);
            Vector3 nextDir = Flatten(path[currentWaypointIndex + 1] - currentWaypoint);
            float angleBetween = Vector3.Angle(currentDir, nextDir);
            // if angle too sharp 
            if (angleBetween < 70f)
                advanceRadius = 22f;
        }

        // Only go if roughly facing toward the waypoint
        if (currentWaypointIndex < path.Count - 1 && distance < advanceRadius)
        {

            float angleToWaypoint = Vector3.SignedAngle(flatForward, flatDesired, Vector3.up);
            if (Mathf.Abs(angleToWaypoint) < 90f)
            {
                currentWaypointIndex++;
                currentWaypoint = path[currentWaypointIndex];
                toWaypoint = currentWaypoint - transform.position;
                dirToWaypoint = toWaypoint.normalized;
                desiredDirection = dirToWaypoint;
                flatDesired = Flatten(desiredDirection);
                distance = toWaypoint.magnitude;
            }
        }

        // Final check at the end waypoint(basically how far it takes to stop at the end)
        float stopRadius = 5f;
        bool atFinal = (currentWaypointIndex == path.Count - 1);
        if (atFinal && distance < stopRadius)
        {
            kinematic.SetDesiredSpeed(0f);
            kinematic.SetDesiredRotationalVelocity(0f);
            return;
        }

        // Rotation and speed
        float angle = Vector3.SignedAngle(flatForward, flatDesired, Vector3.up);
        float angleAbs = Mathf.Abs(angle);
        float maxRot = kinematic.GetMaxRotationalVelocity();
        float rotSpeed = 0f;

        // Reverse + turn for U-turns
        if (angleAbs > 165f)
        {
            float reverseSpeed = -kinematic.GetMaxSpeed() * 0.4f;
            rotSpeed = Mathf.Clamp(angle * 2f, -maxRot, maxRot);

            kinematic.SetDesiredSpeed(reverseSpeed);
            kinematic.SetDesiredRotationalVelocity(rotSpeed);

            return;
        }
        //else Move forward slowly while turning
        else if (angleAbs > 90f)
        {
            float turningSpeed = kinematic.GetMaxSpeed() * 0.2f;
            rotSpeed = Mathf.Clamp(angle * 2f, -maxRot, maxRot);

            kinematic.SetDesiredSpeed(turningSpeed);
            kinematic.SetDesiredRotationalVelocity(rotSpeed);

            return;
        }

        //forward movement
        float turnSharpness = Mathf.Clamp01(angleAbs / 90f);
        float slowFactor = Mathf.Lerp(1f, 0.3f, turnSharpness);
        float speed = kinematic.GetMaxSpeed() * slowFactor;

        rotSpeed = Mathf.Clamp(angle * 3f, -maxRot, maxRot);

        kinematic.SetDesiredSpeed(speed);
        kinematic.SetDesiredRotationalVelocity(rotSpeed);

        if (label != null)
            label.text = $"Dist: {distance:F2}";
    }


    public void SetTarget(Vector3 target)
    {
        this.target = target;
        EventBus.ShowTarget(target);
    }

    public void SetPath(List<Vector3> path)
    {
        this.path = path;
        currentWaypointIndex = 0;
    }

    public void SetMap(List<Wall> outline)
    {
        this.path = null;
        this.target = transform.position;
    }
}
