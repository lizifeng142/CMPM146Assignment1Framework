using UnityEngine;
using System.Collections.Generic;
using TMPro;

public class SteeringBehavior : MonoBehaviour
{
    public Vector3 target;
    public KinematicBehavior kinematic;
    public List<Vector3> path;
    // you can use this label to show debug information,
    // like the distance to the (next) target
    public TextMeshProUGUI label;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        kinematic = GetComponent<KinematicBehavior>();
        target = transform.position;
        path = null;
        EventBus.OnSetMap += SetMap;
    }

    // (left click to get a point and starts Seek Target)
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

        if (path == null)
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


    public void SetTarget(Vector3 target)
    {
        this.target = target;
        EventBus.ShowTarget(target);
    }

    public void SetPath(List<Vector3> path)
    {
        this.path = path;
    }

    public void SetMap(List<Wall> outline)
    {
        this.path = null;
        this.target = transform.position;
    }
}
