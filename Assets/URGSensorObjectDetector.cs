using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace HKY
{

    public class Segment : MonoBehaviour
    {
        public List<double> curvature;
        public List<Vector2> points;
        public bool bValid = true;
        public List<double> curvRate;
        public double curvRateMean = 0.0;
        public double curvRateStd = 0.0;
        public int loSrcIndex = -1;
        public int hiSrcIndex = -1;
        public bool bFoundSource = false;
        public bool bFlagDelete = false;
        [SerializeField] public GameObject center;

        public Segment()
        {
            this.curvature = new List<double>();
            this.points = new List<Vector2>();
            this.curvRate = new List<double>();
        }

        //~Segment()
        //{
        //    if (this.center != null)
        //    {
        //        Destroy(this.center);
        //    }
        //}

        public void calcCurvRate()
        {
            this.curvRate.Clear();
            for (int i = 0; i < this.curvature.Count-1; i++)
            {
                double rate = this.curvature[i + 1] - this.curvature[i];
                this.curvRate.Add(rate);
            }

            // Calculate the mean
            this.curvRateMean = 0.0;
            for (int i = 0; i < this.curvRate.Count; i++)
            {
                this.curvRateMean += this.curvRate[i] / (double)this.curvRate.Count;
            }
            // Calculate the standard deviation
            this.curvRateStd = 0.0;
            for (int i = 0; i < this.curvRate.Count; i++)
            {
                double diff = this.curvRate[i] - this.curvRateMean;
                this.curvRateStd += diff*diff / (double)this.curvRate.Count;
            }
            this.curvRateStd = Math.Sqrt(this.curvRateStd);
        }

        public void updateCenter(float viewScale, double ballDiameter)
        {
            if (this.points.Count > 0)
            {
                if (this.center == null)
                {
                    this.center = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    this.center.transform.localScale = new Vector3(5, 5, 5);
                }

                // Calculate the center assuming it represents a spherical objec
                Vector3 cen = new Vector3(0, 0, 0);
                double DEG2RAD = Math.PI / 180.0;
                for (int i = 0; i < this.points.Count-1; i++)
                {
                    // Find the distance to the next point
                    double dist = Vector2.Distance(points[i + 1], points[i]);

                    // Find the radius to the center of the circle
                    //double radius = dist / Math.Cos((90 - Math.Abs(this.curvature[i]/2.0)) * DEG2RAD); // use known radius instead
                    double radius = ballDiameter / 2.0;

                    // Find the vector to the next point
                    Vector2 toNextTmp = points[i + 1] - points[i];
                    Vector3 toNext = new Vector3(toNextTmp.x, toNextTmp.y, 0);
                    // Find the vector that points outward toward us
                    Vector3 outVec = new Vector3(0, 0, 1);
                    // Find the vector that points toward the center
                    Vector3 towardCenter = Vector3.Cross(outVec, toNext).normalized;
                    // Find the Vector representing the source point
                    Vector3 originPt = new Vector3(points[i].x, points[i].y, 0);
                    // Calculate the predicted center
                    Vector3 predCen = towardCenter * (float)radius + originPt;
                    // Add this center
                    cen += predCen / (float)(this.points.Count - 1);
                }

                this.center.transform.position = cen * viewScale; // new Vector3(points[0].x * viewScale, points[0].y * viewScale, 0);
                float ballSize = (float)ballDiameter * (float)viewScale;
                this.center.transform.localScale = new Vector3(ballSize, ballSize, ballSize);
                this.center.GetComponent<Renderer>().material.color = new Color(0, 0, 1, 0.5F);

                this.center.SetActive(this.bValid);
            }
        }

    }


    // http://sourceforge.net/p/urgnetwork/wiki/top_jp/
    // https://www.hokuyo-aut.co.jp/02sensor/07scanner/download/pdf/URG_SCIP20.pdf
    public class URGSensorObjectDetector : MonoBehaviour
    {

        [Header("Connection with Sensor")]
        [SerializeField] string ip_address = "192.168.0.10";
        [SerializeField] int port_number = 10940;
        UrgDeviceEthernet urg;
        public int sensorScanSteps { get; private set; }
        public bool open { get; private set; }
        bool gd_loop = false;

        [Header("Detection Area Constrain")]
        public DistanceCroppingMethod distanceCroppingMethod = DistanceCroppingMethod.RADIUS;
        long[] distanceConstrainList;

        [Header("----------Rect based constrain")]
        public int detectRectWidth;     //Unit is MM
        public int detectRectHeight;    //Unit is MM
        Rect detectAreaRect
        {
            get
            {
                Rect rect = new Rect(0, 0, detectRectWidth, detectRectHeight);
                rect.x -= (detectRectWidth / 2);
                return rect;
            }
        }

        [Header("----------Radius based constrain")]
        public long maxDetectionDist = 7000;//for radius based detection, unit is mm

        [Header("Post Processing Distance Data")]
        [SerializeField] bool smoothDistanceCurve = false;
        [SerializeField] bool smoothDistanceByTime = false;
        //if change between two consecutive frame is bigger than this number, then do not do smoothing
        [Range(1, 500)] public int timeSmoothBreakingDistanceChange = 200;
        [Range(1, 130)] public int smoothKernelSize = 21;
        List<long> smoothByTimePreviousList = new List<long>();
        [Range(0.01f, 1f)] public float timeSmoothFactor;

        [Header("Points")]
        [SerializeField] List<double> dists; // meters
        [SerializeField] List<Vector2> points; // meters
        [SerializeField] List<GameObject> pointsObj; // meters * pointsViewScale
        [SerializeField] public float pointsViewScale = 10.0F;
        [SerializeField] public float pointsViewRadius = 10.0F;
        [SerializeField] public float pointsRotationOffset = 180.0F; // degrees
        [SerializeField] public bool bFlipPointsX = true;
        [SerializeField] public bool bFlipPointsY = false;
        [SerializeField] public bool bSmoothDist = false;
        [SerializeField] public int smoothDistRadius = 3;
        [SerializeField] public bool bSmoothPts = false;
        [SerializeField] public int smoothPtsRadius = 3;
        [SerializeField] public bool bFlipCurvatureSign = false;
        [SerializeField] public bool bRemoveConvexities = true;
        [SerializeField] public bool bSmoothCurv = true;
        [SerializeField] public int smoothCurvRadius = 3;
        [SerializeField] public double ballDiameter = 0.22; // meters
        [SerializeField] public double curvThreshLoFrac = 0.75;
        [SerializeField] public double curvThreshHiFrac = 4.0;
        [SerializeField] public int maxOutlierSize = 2; // TODO: dependent on distance away
        [SerializeField] public int maxSpanGapSize = 1; // TODO: dependent on distance away
        [SerializeField] public int removeNumSegmentEdgeSamples = 2; // TODO: dependent on distance away
        [SerializeField] List<Segment> segments;
        [SerializeField] public int minSegmentLength = 5;
        [SerializeField] public double curvRateMeanThresh = 2.0;
        [SerializeField] public double curvRateStdThresh = 4.0;



        // TODO: crop rectangular space
        // TODO: remove far and near values


        [Header("Object Detection")]
        [Range(1, 40)] public int noiseLimit = 7;
        [Range(10, 1000)] public int deltaLimit = 200;
        List<RawObject> rawObjectList;


         [SerializeField] List<ProcessedObject> detectedObjects;
        [Header("Object Tracking")]
        [SerializeField] float distanceThresholdForMerge = 300;
        [Range(0.01f, 1f)] public float objectPositionSmoothTime = 0.2f;
        public Vector2 positionOffset;
        public bool useOffset = true;

        //Events
        public static System.Action<ProcessedObject> OnNewObject;
        public static System.Action<ProcessedObject> OnLostObject;

        [Header("Debug Draw")]
        [SerializeField] bool debugDrawDistance = false;
        [SerializeField] bool drawObjectRays = true;
        [SerializeField] bool drawObjectCenterRay = true;
        [SerializeField] bool drawObject = true;
        [SerializeField] bool drawProcessedObject = true;
        [SerializeField] bool drawRunningLine = true;
        [SerializeField] bool showHardwareControlButtons = false;
        [SerializeField] bool recalculateConstrainAreaEveryFrame = false;
        //colors
        [SerializeField] Color distanceColor = Color.white;
        [SerializeField] Color strengthColor = Color.red;
        [SerializeField] Color objectColor = Color.green;
        [SerializeField] Color processedObjectColor = Color.cyan;

        //General
        List<long> croppedDistances;
        List<long> strengths;
        Vector3[] directions;

        public enum DistanceCroppingMethod
        {
            RECT, RADIUS
        }

        public ProcessedObject GetObjectByGuid(Guid guid)
        {
            ProcessedObject o = null;
            foreach (var obj in detectedObjects)
            {
                if (obj.guid == guid)
                {
                    o = obj;
                }
            }
            if (o == null) Debug.LogWarning("cannot find object with guid " + guid);
            return o;
        }

        public List<ProcessedObject> GetObjects(float ageFilter = 0.5f)
        {
            var o = from obj in detectedObjects
                    where obj.age > ageFilter
                    select obj;
            return o.ToList();
        }

        void CalculateDistanceConstrainList(int steps)
        {
            switch (distanceCroppingMethod)
            {
                case DistanceCroppingMethod.RADIUS:
                    for (int i = 0; i < steps; i++)
                    {
                        if (directions[i].y < 0)
                        {
                            distanceConstrainList[i] = 0;
                        }
                        else
                        {
                            distanceConstrainList[i] = maxDetectionDist;
                        }
                    }
                    break;

                case DistanceCroppingMethod.RECT:
                    float keyAngle = Mathf.Atan(detectRectHeight / (detectRectWidth / 2f));

                    for (int i = 0; i < steps; i++)
                    {
                        if (directions[i].y <= 0)
                        {
                            distanceConstrainList[i] = 0;
                        }
                        else
                        {
                            float a = Vector3.Angle(directions[i], Vector3.right) * Mathf.Deg2Rad;
                            float tanAngle = Mathf.Tan(a);
                            float pn = tanAngle / Mathf.Abs(tanAngle);

                            float r = 0;
                            if (a < keyAngle || a > Mathf.PI - keyAngle)
                            {
                                float x = pn * detectRectWidth / 2;
                                float y = x * Mathf.Tan(a);
                                r = y / Mathf.Sin(a);
                            }
                            else if (a >= keyAngle && a <= Mathf.PI - keyAngle)
                            {
                                float angle2 = Mathf.PI / 2 - a;
                                float y = detectRectHeight;
                                float x = y * Mathf.Tan(angle2);
                                r = x / Mathf.Sin(angle2);
                            }

                            if (r < 0 || float.IsNaN(r))
                            {
                                r = 0;
                            }

                            distanceConstrainList[i] = (long)r;
                        }


                    }

                    break;
            }
        }

        List<long> ConstrainDetectionArea(List<long> beforeCrop, DistanceCroppingMethod method)
        {
            List<long> result = new List<long>();

            for (int i = 0; i < beforeCrop.Count; i++)
            {
                if (beforeCrop[i] > distanceConstrainList[i] || beforeCrop[i] <= 0)
                {
                    result.Add(distanceConstrainList[i]);
                }
                else
                {
                    result.Add(beforeCrop[i]);
                }
            }

            return result;
        }

#if UNITY_EDITOR
        //todo: deal with offfset
        private void OnDrawGizmos()
        {
            //draw boundary
            switch (distanceCroppingMethod)
            {
                case DistanceCroppingMethod.RADIUS:
                    Gizmos.DrawWireSphere(new Vector3(0, 0, 0) + transform.position, maxDetectionDist);
                    break;
                case DistanceCroppingMethod.RECT:
                    Gizmos.DrawWireCube(new Vector3(0, detectRectHeight / 2, 0) + transform.position, new Vector3(detectAreaRect.width, detectAreaRect.height, 1));
                    break;
            }

            //draw distance rays
            if (debugDrawDistance && croppedDistances != null)
            {
                for (int i = 0; i < croppedDistances.Count; i++)
                {
                    Vector3 dir = directions[i];
                    long dist = croppedDistances[i];
                    Debug.DrawLine(Vector3.zero + transform.position, (dist * dir) + transform.position, distanceColor);
                }
            }
            //draw raw objects
            if (rawObjectList == null) return;
            for (int i = 0; i < rawObjectList.Count; i++)
            {
                var obj = rawObjectList[i];
                if (obj.idList.Count == 0 || obj.distList.Count == 0) return;
                Vector3 dir = directions[obj.medianId];
                long dist = obj.medianDist;
                if (drawObjectRays)
                {
                    for (int j = 0; j < obj.distList.Count; j++)
                    {
                        var myDir = directions[obj.idList[j]];
                        Debug.DrawLine(Vector3.zero + transform.position, (myDir * obj.distList[j]) + transform.position, objectColor);
                    }
                }
                //center
                if (drawObjectCenterRay) Debug.DrawLine(Vector3.zero + transform.position, (dir * dist) + transform.position, Color.blue);
                //draw objects!
                if (drawObject) Gizmos.DrawWireCube((Vector3)obj.position + transform.position, new Vector3(100, 100, 0));
            }

            if (drawProcessedObject)
            {
                //draw processed object
                foreach (var pObj in detectedObjects)
                {
                    Gizmos.color = processedObjectColor;
                    float size = 30;// pObj.size;
                    Gizmos.DrawCube(pObj.position, new Vector3(size, size, 1));
                    UnityEditor.Handles.Label(pObj.position, pObj.position.ToString());
                }
            }



            if (drawRunningLine)
            {
                for (int i = 1; i < croppedDistances.Count; i++)
                {
                    Gizmos.DrawLine(new Vector3(i, detectRectHeight + croppedDistances[i], 0), new Vector3(i - 1, detectRectHeight + croppedDistances[i - 1], 0));
                }
            }
        }
#endif


        private void OnGUI()
        {
            if (!showHardwareControlButtons) return;
            // https://sourceforge.net/p/urgnetwork/wiki/scip_jp/
            if (GUILayout.Button("VV: (Get Version Information)"))
            {
                urg.Write(SCIP_library.SCIP_Writer.VV());
            }
            //		if(GUILayout.Button("SCIP2")){
            //			urg.Write(SCIP_library.SCIP_Writer.SCIP2());
            //		}
            if (GUILayout.Button("PP: (Get Parameters)"))
            {
                urg.Write(SCIP_library.SCIP_Writer.PP());
            }
            if (GUILayout.Button("MD: (Measure and Transimission)"))
            {
                urg.Write(SCIP_library.SCIP_Writer.MD(0, 1080, 1, 0, 0));
            }
            if (GUILayout.Button("ME: (Measure Distance and Strength"))
            {
                urg.Write(SCIP_library.SCIP_Writer.ME(0, 1080, 1, 1, 0));
                open = true;
            }
            if (GUILayout.Button("BM: (Emit Laser)"))
            {
                urg.Write(SCIP_library.SCIP_Writer.BM());
            }
            if (GUILayout.Button("GD: (Measure Distance)"))
            {
                urg.Write(SCIP_library.SCIP_Writer.GD(0, 1080));
            }
            if (GUILayout.Button("GD_loop"))
            {
                gd_loop = !gd_loop;
            }
            if (GUILayout.Button("QUIT"))
            {
                urg.Write(SCIP_library.SCIP_Writer.QT());
            }

            //GUILayout.Label("distances.Count: " + distances.Count + " / strengths.Count: " + strengths.Count);
            //        GUILayout.Label("distances.Length: " + distances + " / strengths.Count: " + strengths.Count);
            // GUILayout.Label("drawCount: " + drawCount + " / detectObjects: " + detectedObjects.Count);
        }

        // Use this for initialization
        private void Start()
        {
            croppedDistances = new List<long>();
            strengths = new List<long>();
            detectedObjects = new List<ProcessedObject>();

            urg = gameObject.AddComponent<UrgDeviceEthernet>();
            urg.StartTCP(ip_address, port_number);

            StartMeasureDistance();
        }

        private void StartMeasureDistance()
        {
            urg.Write(SCIP_library.SCIP_Writer.MD(0, 1080, 1, 0, 0));
        }

        private void CacheDirections()
        {
            float d = Mathf.PI * 2 / 1440;
            float offset = d * 540;
            directions = new Vector3[sensorScanSteps];
            for (int i = 0; i < directions.Length; i++)
            {
                float a = d * i + offset;
                directions[i] = new Vector3(-Mathf.Cos(a), -Mathf.Sin(a), 0);
            }
        }


        private void Update()
        {

            if (smoothKernelSize % 2 == 0) { smoothKernelSize += 1; }


            List<long> originalDistances = new List<long>();
            lock (urg.distances)
            {
                if (urg.distances.Count <= 0) return;
                originalDistances = new List<long>(urg.distances);
            }
            if (originalDistances.Count <= 0) return;


            //Setting up things, one time
            if (sensorScanSteps <= 0)
            {
                sensorScanSteps = urg.distances.Count;
                distanceConstrainList = new long[sensorScanSteps];
                CacheDirections();
                CalculateDistanceConstrainList(sensorScanSteps);

                //points = new List<Vector2>(urg.distances.Count);

            }

            if (recalculateConstrainAreaEveryFrame)
            {
                if (Time.frameCount % 10 == 0)
                {
                    CalculateDistanceConstrainList(sensorScanSteps);
                }
            }

            if (gd_loop)
            {
                urg.Write(SCIP_library.SCIP_Writer.GD(0, 1080));
            }

            var cropped = ConstrainDetectionArea(originalDistances, distanceCroppingMethod);
            croppedDistances.Clear();
            croppedDistances.AddRange(cropped);


            if (smoothDistanceCurve)
            {
                croppedDistances = SmoothDistanceCurve(croppedDistances, smoothKernelSize);
            }
            if (smoothDistanceByTime)
            {
                croppedDistances = SmoothDistanceCurveByTime(croppedDistances, ref smoothByTimePreviousList, timeSmoothFactor);
            }

            //-----------------
            //  detect objects


            UpdateObjectList();



            // ----------------------------
            // Update 2D Points

            // Fill the vector of points if they don't exist
            if (originalDistances.Count != points.Count)
            {
                dists.Clear();
                points.Clear();
                pointsObj.Clear();
                for (int i = 0; i < originalDistances.Count; i++)
                {
                    dists.Add(0.0);

                    points.Add(new Vector2(0, 0));

                    GameObject pointObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    pointObj.transform.localScale = new Vector3(pointsViewRadius, pointsViewRadius, pointsViewRadius);
                    pointObj.transform.position = new Vector3(points[i].x, points[i].y, 0);
                    pointsObj.Add(pointObj);

                    segments = new List<Segment>();
                }
            }

            // Convert the distances to meters
            for (int i = 0; i < originalDistances.Count; i++)
            {
                dists[i] = (double)originalDistances[i] / 1000.0;
            }

            // Don't proceed with CV if there are no points observed
            if (dists.Count == 0) return;

            // TODO: Apply Median Filter

            // Smooth distances if necessary
            if (bSmoothDist)
            {
                // Calculate the smoothed distances
                List<double> distsSmoothed = new List<double>();
                for (int i = 0; i < dists.Count; i++)
                {
                    int lo = Math.Max(i - smoothDistRadius, 0);
                    int hi = Math.Min(i + smoothDistRadius + 1, dists.Count);
                    int nSamples = hi - lo;
                    double mult = 1.0 / (double)nSamples;
                    double avgDist = 0.0;
                    for (int j = lo; j < hi; j++)
                    {
                        avgDist += dists[j] * mult;
                    }
                    distsSmoothed.Add(avgDist);
                }

                // Copy the smoothed distances back into the original vector
                for (int i = 0; i < dists.Count; i++)
                {
                    dists[i] = distsSmoothed[i];
                }
            }

            // Update all the points given the recent distances
            double degreeStep = 270.0 / (double)originalDistances.Count;
            double DEG2RAD = Math.PI / 180.0;
            for (int i = 0; i < dists.Count; i++)
            {
                double angleRad = (-45.0 + (double)i * degreeStep + (double)pointsRotationOffset) * DEG2RAD;
                double distM = dists[i];
                double x = (bFlipPointsX ? -1.0 : 1.0) * Math.Cos(angleRad) * distM;
                double y = (bFlipPointsY ? -1.0 : 1.0) * Math.Sin(angleRad) * distM;
                points[i] = new Vector2((float)x, (float)y);
            }

            // TODO: Smooth the points?

            // Calculate angle between pairs of points
            bool[] validCurvature = new bool[dists.Count];
            double[] curvature = new double[dists.Count];
            for (int i = 0; i < dists.Count; i++)
            {
                validCurvature[i] = !(i == 0 || i == (dists.Count - 1));
                curvature[i] = 0;
                if (!validCurvature[i]) continue;

                double angle = Vector2.SignedAngle(points[i - 1] - points[i], points[i] - points[i + 1]);
                angle *= (bFlipCurvatureSign ? -1.0 : 1.0);
                curvature[i] = angle;
            }

            // Smooth the curvature
            if (bSmoothCurv)
            {
                // Calculate the smoothed distances
                double[] curvSmoothed = new double[dists.Count];
                for (int i = 0; i < dists.Count; i++)
                {
                    int lo = Math.Max(i - smoothCurvRadius, 0);
                    int hi = Math.Min(i + smoothCurvRadius + 1, dists.Count);
                    double sum = 0.0;
                    int sumWeights = 0;
                    for (int j = lo; j < hi; j++)
                    {
                        if (!validCurvature[j]) continue;
                        sum += curvature[j];
                        sumWeights += 1;
                    }
                    double avg = (sumWeights > 0) ? (sum / (double)sumWeights) : 0.0;
                    curvSmoothed[i] = avg;
                }

                // Copy the smoothed distances back into the original vector
                for (int i = 0; i < dists.Count; i++)
                {
                    curvature[i] = curvSmoothed[i];
                }
            }

            // Remove all convexities
            if (bRemoveConvexities)
            {
                for (int i = 0; i < dists.Count; i++)
                {
                    if (!validCurvature[i]) continue;
                    if (curvature[i] < 0) validCurvature[i] = false;
                }
            }

            // Remove all angles below the low threshold and above the high threshold 
            // (as it relates to the radius of the ball)
            for (int i = 0; i < dists.Count; i++)
            {
                if (!validCurvature[i]) continue;

                double idealAngle = dists[i] / (4.0 * ballDiameter);
                double loThresh = idealAngle * curvThreshLoFrac;
                double hiThresh = idealAngle * curvThreshHiFrac;
                if (curvature[i] < loThresh || curvature[i] > hiThresh) validCurvature[i] = false; // set to 0?
            }

            // Remove outiers (segments with less thana certain number of values)
            contractBool(maxOutlierSize, dists.Count, validCurvature);
            dilateBool(maxOutlierSize, dists.Count, validCurvature);

            // TODO: Contract, Dilate to fill in holes and remove single outliers
            dilateBool(maxSpanGapSize, dists.Count, validCurvature);
            contractBool(maxSpanGapSize, dists.Count, validCurvature);
            // Remove this many bools at the edges
            for (int i = 0; i < maxSpanGapSize; i++)
            {
                validCurvature[i] = false;
                validCurvature[dists.Count - 1 - i] = false;
            }

            // Remove the segment's edges, since these are usually inaccurate
            contractBool(removeNumSegmentEdgeSamples, dists.Count, validCurvature);





            // Extract all segments that satisfy these parameters
            // TODO: Break up segments with attached arms/legs in view?

            // Create new segments from this frame
            List<Segment> tmpSegments = new List<Segment>();
            for (int i = 1; i < dists.Count; i++)
            {
                if (!validCurvature[i-1] && validCurvature[i])
                {
                    // Begin a new segment
                    Segment seg = gameObject.AddComponent<Segment>();
                    seg.loSrcIndex = i;
                    tmpSegments.Add(seg);
                }
                if (validCurvature[i])
                {
                    tmpSegments[tmpSegments.Count - 1].points.Add(points[i]);
                    tmpSegments[tmpSegments.Count - 1].curvature.Add(curvature[i]);
                    tmpSegments[tmpSegments.Count - 1].hiSrcIndex = i;
                }
            }

            // Check if these segments are the same as the previous frame's segments
            for (int i = 0; i < segments.Count; i++)
            {
                segments[i].bFlagDelete = true;
            }
            for (int i = 0; i < segments.Count; i++)
            {
                for (int j = 0; j < tmpSegments.Count; j++)
                {
                    if (tmpSegments[j].bFoundSource) continue;
                    if (segments[i].hiSrcIndex < tmpSegments[j].loSrcIndex || segments[i].loSrcIndex > tmpSegments[j].hiSrcIndex)
                    {
                        // not in range
                        continue;
                    } else
                    {
                        // in range
                        tmpSegments[j].bFoundSource = true;
                        segments[i].bFlagDelete = false;
                        // copy the new data over
                        segments[i].curvature.Clear();
                        segments[i].curvature.AddRange(tmpSegments[j].curvature);
                        segments[i].points.Clear();
                        segments[i].points.AddRange(tmpSegments[j].points);
                        segments[i].curvRate.Clear();
                    }
                }
            }
            // Delete any old segments
            for (int i = segments.Count-1; i >= 0; i--)
            {
                if (segments[i].bFlagDelete)
                {
                    if (segments[i].center != null) Destroy(segments[i].center);
                    Destroy(segments[i]);
                    segments.RemoveAt(i);
                }
            }
            // Add any new segments
            for (int i = tmpSegments.Count-1; i>= 0; i--)
            {
                if (tmpSegments[i].bFoundSource)
                {
                    if (tmpSegments[i].center != null) Destroy(tmpSegments[i].center);
                    Destroy(tmpSegments[i]);
                } else
                {
                    segments.Add(tmpSegments[i]);
                }
            }
            tmpSegments.Clear();


            // Remove segments with too few points
            for (int i = 0; i < segments.Count; i++)
            {
                if (segments[i].points.Count < minSegmentLength)
                {
                    segments[i].bValid = false;
                }
            }

            // Calculate the rate of curvature change for each segment. 
            // If this change is constant, then the object is circular
            for (int i = 0; i < segments.Count; i++)
            {
                if (Math.Abs(segments[i].curvRateMean) > curvRateMeanThresh || segments[i].curvRateStd > curvRateStdThresh)
                {
                    segments[i].bValid = false;
                }
            }

            // Update the segments' centers
            for (int i = 0; i < segments.Count; i++)
            {
                segments[i].updateCenter(pointsViewScale, ballDiameter);
            }

            // Set all of the debug points' positions
            for (int i = 0; i < dists.Count; i++)
            {
                pointsObj[i].transform.localScale = new Vector3(pointsViewRadius, pointsViewRadius, pointsViewRadius);

                pointsObj[i].transform.position = new Vector3(points[i].x * pointsViewScale, points[i].y * pointsViewScale, 0);
                pointsObj[i].GetComponent<Renderer>().material.color = new Color(1.0F, validCurvature[i] ? 1.0F : 0.0F, 0.0F, 1.0F);
            }



        }

        private void dilateBool(int nDilations, int length, bool[] values)
        {
            bool[] outValues = new bool[length];
            for (int i = 0; i < length; i++)
            {
                int lo = Math.Max(i - nDilations, 0);
                int hi = Math.Min(i + nDilations + 1, length);
                bool val = false;
                for (int j = lo; j < hi; j++)
                {
                    val |= values[j];
                }
                outValues[i] = val;
            }
            
            // copy values to output
            for (int i = 0; i < length; i++)
            {
                values[i] = outValues[i];
            }
        }

        private void contractBool(int nContractions, int length, bool[] values)
        {
            bool[] outValues = new bool[length];
            for (int i = 0; i < length; i++)
            {
                int lo = Math.Max(i - nContractions, 0);
                int hi = Math.Min(i + nContractions + 1, length);
                bool val = true;
                for (int j = lo; j < hi; j++)
                {
                    val &= values[j];
                }
                outValues[i] = val;
            }

            // copy values to output
            for (int i = 0; i < length; i++)
            {
                values[i] = outValues[i];
            }
        }

        private List<long> SmoothDistanceCurve(List<long> croppedDistances, int smoothKernelSize)
        {
            HKY.MovingAverage movingAverageFilter = new MovingAverage();
            return movingAverageFilter.Filter(croppedDistances.ToArray(), smoothKernelSize).ToList();
        }

        private List<RawObject> DetectObjects(List<long> croppedDistances, long[] distanceConstrainList)
        {
            if (directions.Length <= 0)
            {
                Debug.LogError("directions array is not setup.");
                return new List<RawObject>();
            }

            int objectId = 0;

            var resultList = new List<RawObject>();
            bool isGrouping = false;
            for (int i = 1; i < croppedDistances.Count - 1; i++)
            {

                float deltaA = Mathf.Abs(croppedDistances[i] - croppedDistances[i - 1]);
                float deltaB = Mathf.Abs(croppedDistances[i + 1] - croppedDistances[i]);

                var dist = croppedDistances[i];
                var ubDist = distanceConstrainList[i];

                //
                if (dist < ubDist && (deltaA < deltaLimit && deltaB < deltaLimit))
                {
                    if (!isGrouping)
                    {
                        //is not grouping
                        isGrouping = true;
                        //start a new object and start grouping
                        RawObject newObject = new RawObject(directions, objectId++);
                        newObject.idList.Add(i);
                        newObject.distList.Add(dist);
                        resultList.Add(newObject);
                    }
                    else
                    {
                        //already grouping, add stuff to existing group
                        var newObject = resultList[resultList.Count - 1];
                        newObject.idList.Add(i);
                        newObject.distList.Add(dist);
                    }

                }
                else
                {
                    if (isGrouping)
                    {
                        isGrouping = false;
                    }
                }
            }
            //remove the ones that might be noise
            resultList.RemoveAll(item => item.idList.Count < noiseLimit);
            //all finished, calculate position and save it for later use
            resultList.ForEach(i => { i.GetPosition(); });
            return resultList;
        }


        void UpdateObjectList()
        {
            List<HKY.RawObject> newlyDetectedObjects = DetectObjects(croppedDistances, distanceConstrainList);
            
            //apply offset!===============
            if (useOffset)
            {
                foreach (var obj in newlyDetectedObjects)
                {
                    obj.position += positionOffset;
                }
            }
            //============================

            rawObjectList = new List<RawObject>(newlyDetectedObjects);

            lock (detectedObjects)
            {
                //update existing objects
                if (detectedObjects.Count != 0)
                {
                    foreach (var oldObj in detectedObjects)
                    {
                        Dictionary<RawObject, float> objectByDistance = new Dictionary<RawObject, float>();
                        //calculate all distance between existing objects and newly found objects
                        foreach (var newObj in newlyDetectedObjects)
                        {
                            float distance = Vector3.Distance(newObj.position, oldObj.position);
                            objectByDistance.Add(newObj, distance);
                        }

                        if (objectByDistance.Count <= 0)
                        {
                            oldObj.Update();
                        }
                        else
                        {
                            //find the closest new obj and check if the dist is smaller than distanceThresholdForMerge, if yes, then update oldObj's position to this newObj
                            var closest = objectByDistance.Aggregate((l, r) => l.Value < r.Value ? l : r);
                            if (closest.Value <= distanceThresholdForMerge)
                            {
                                oldObj.Update(closest.Key.position, closest.Key.size);
                                //remove the newObj that is being used
                                newlyDetectedObjects.Remove(closest.Key);
                            }
                            else
                            {
                                //this oldObj cannot find a new one that is close enough to it
                                oldObj.Update();
                            }
                        }

                    }

                    //remove all missed objects
                    for (int i = 0; i < detectedObjects.Count; i++)
                    {
                        var obj = detectedObjects[i];
                        if (obj.clear)
                        {
                            detectedObjects.RemoveAt(i);
                            if (OnLostObject != null) { OnLostObject(obj); }
                        }
                    }

                    //create new object for those newobject that cannot find match from the old objects
                    foreach (var leftOverNewObject in newlyDetectedObjects)
                    {
                        var newbie = new ProcessedObject(leftOverNewObject.position, leftOverNewObject.size, objectPositionSmoothTime);
                        detectedObjects.Add(newbie);
                        if (OnNewObject != null) { OnNewObject(newbie); }
                    }
                }
                else //add all raw objects into detectedObjects
                {
                    foreach (var obj in rawObjectList)
                    {
                        var newbie = new ProcessedObject(obj.position, obj.size, objectPositionSmoothTime);
                        detectedObjects.Add(newbie);
                        if (OnNewObject != null) { OnNewObject(newbie); }
                    }
                }
            }
        }



        List<long> SmoothDistanceCurveByTime(List<long> newList, ref List<long> previousList, float smoothFactor)
        {
            if (previousList.Count <= 0)
            {
                previousList = newList;
                return newList;
            }
            else
            {
                long[] result = new long[newList.Count];
                for (int i = 0; i < result.Length; i++)
                {

                    float diff = newList[i] - previousList[i];
                    if (diff > timeSmoothBreakingDistanceChange)
                    {
                        result[i] = newList[i];
                        previousList[i] = result[i];
                    }
                    else
                    {
                        float smallDiff = diff * smoothFactor;
                        float final = previousList[i] + smallDiff;

                        result[i] = (long)final;
                        previousList[i] = result[i];
                    }
                }
                return result.ToList();
            }

        }
        // PP
        //	MODL ... 传感器信号类型
        //	DMIN ... 最小計測可能距離 (mm)
        //	DMAX ... 最大計測可能距離 (mm)
        //	ARES ... 角度分解能(360度の分割数)
        //	AMIN ... 最小可测量方向值
        //	AMAX ... 最大可测量方向值
        //	AFRT ... 正面方向値
        //	SCAN ... 標準操作角速度
    }
}