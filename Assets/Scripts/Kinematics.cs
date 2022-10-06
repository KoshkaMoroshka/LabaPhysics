using System;
using UnityEngine;
using Random = UnityEngine.Random;


public class Kinematics : MonoBehaviour
{
    private SingleDegreeJoint[] _joints;
    public float[] solution;
    public float[] currentSolution;
    public float[] beststate;
    public float prevdist = float.MaxValue;

    [SerializeField] private GameObject actor;
    [SerializeField] private GameObject target;
    [SerializeField] private float maxDegreeAngle = 90;

    private float _bestDistanceToTarget;

    private float _currentDistanceToTarget;
    private float _previousDistanceToTarget;

    // Start is called before the first frame update
    private void Start()
    {
        _joints = GetComponentsInChildren<SingleDegreeJoint>();
        solution = new float[_joints.Length];
        currentSolution = new float[_joints.Length];
        beststate = new float[_joints.Length];
        for (var i = 0; i < _joints.Length; i++)
        {
            beststate[i] = _joints[i].GetValue();
        }
    }

    private float DistanceActorToTarget()
    {
        return Vector3.Distance(actor.transform.position, target.transform.position);
    }

    private void Update()
    {
        if (DistanceActorToTarget() < 0.3f)
        {
            rotationSpeed = 1;
            return;
        }

        Randoms(beststate);
        var newdist = InverseKinematics(beststate);
        if (newdist < prevdist)
        {
            for (var i = 0; i < beststate.Length; i++)
                solution[i] = beststate[i];
            prevdist = newdist;
            speedDec();
        }
        else
        {
            for (var i = 0; i < beststate.Length; i++)
                beststate[i] = solution[i];
            speedInc();
        }

        ForwardKinematics();
        prevdist = InverseKinematics(currentSolution);
    }

    private float InverseKinematics(float[] solutions)
    {
        for (var i = 0; i < solutions.Length; i++)
        {
            _joints[i].SetValue(solutions[i]);
        }
        return DistanceActorToTarget();
    }

    private void ForwardKinematics()
    {
        for (var i = 0; i < solution.Length; i++)
        {
            _joints[i].SetValue(currentSolution[i] = currentSolution[i] * 0.9f + solution[i] * 0.1f);
        }
    }


    private float _maxRotationSpeed = 20;
    private float _rotationAcceleration = 0.001f;
    public float rotationSpeed = 1;

    void speedInc()
    {
        rotationSpeed += (_maxRotationSpeed - rotationSpeed) * _rotationAcceleration;
    }

    void speedDec()
    {
        rotationSpeed *= 1 - _rotationAcceleration;
    }

    private void Randoms(float[] solutions)
    {
        for (var i = 0; i < solutions.Length; i++)
        {
            solutions[i] = InRange(solutions[i] + (Random.value * 2 - 1) * rotationSpeed);
        }
    }


    private float InRange(float value)
    {
        return value < -maxDegreeAngle ? -maxDegreeAngle : value > maxDegreeAngle ? maxDegreeAngle : value;
    }
}