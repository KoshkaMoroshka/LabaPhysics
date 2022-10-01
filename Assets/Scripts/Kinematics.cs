using UnityEngine;


public class Kinematics : MonoBehaviour
{
    private SingleDegreeJoint[] _joints;
    public float[] solution;

    [SerializeField] private GameObject actor;
    [SerializeField] private GameObject target;
    [SerializeField] private float maxDegreeAngle = 90;

    private float _bestDistanceToTarget;
    [SerializeField] private float _minDistanceToObject = 0.5f;

    private float _currentDistanceToTarget;
    private float _previousDistanceToTarget;

    // Start is called before the first frame update
    private void Start()
    {
        _joints = GetComponentsInChildren<SingleDegreeJoint>();
        solution = new float[_joints.Length];
        _bestDistanceToTarget = DistanceActorToTarget(actor.transform.position, target.transform.position);
        for (var i = 0; i < _joints.Length; i++)
        {
            solution[i] = _joints[i].GetValue();
        }

        _currentDistanceToTarget = DistanceActorToTarget(actor.transform.position, target.transform.position);
    }

    private float DistanceActorToTarget(Vector3 positionActor, Vector3 positionTarget)
    {
        return Vector3.Distance(positionActor, positionTarget);
    }

    private void InverseKinematic()
    {
        float[] lastSolutions = new float[solution.Length];
        for (var i = 0; i < lastSolutions.Length; i++)
        {
            lastSolutions[i] = solution[i];
        }

        Solve();
        ForwardKinematics();
        var distance = DistanceActorToTarget(actor.transform.position, target.transform.position);
        if (distance < _bestDistanceToTarget)
            _bestDistanceToTarget = distance;
        else
        {
            solution = lastSolutions;
            ForwardKinematics();
        }
    }

    // Update is called once per frame
    private void Update()
    {
        _previousDistanceToTarget = _currentDistanceToTarget;
        _currentDistanceToTarget = DistanceActorToTarget(actor.transform.position, target.transform.position);
        //ForwardKinematics();
        if (_currentDistanceToTarget < _minDistanceToObject)
            return;

        if (_previousDistanceToTarget == _currentDistanceToTarget)
            speedInc();
        else
            speedDec();


        _bestDistanceToTarget = DistanceActorToTarget(actor.transform.position, target.transform.position);
        InverseKinematic();
    }

    private void ForwardKinematics()
    {
        for (var i = 0; i < solution.Length; i++)
        {
            _joints[i].SetValue(solution[i]);
        }
    }

    private float _maxRotationSpeed = 180;
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

    private void Solve()
    {
        for (var i = 0; i < solution.Length; i++)
        {
            solution[i] = InRange(solution[i] + (Random.value * 2 - 1) * rotationSpeed);
        }
    }


    private float InRange(float value)
    {
        return value < -maxDegreeAngle ? -maxDegreeAngle : value > maxDegreeAngle ? maxDegreeAngle : value;
    }
}