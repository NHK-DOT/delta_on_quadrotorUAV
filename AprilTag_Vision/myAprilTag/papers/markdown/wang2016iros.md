|                    |     | AprilTag   |           |           | 2: Efficient |         | and   | robust    | fiducial |     | detection |     |     |
| ------------------ | --- | ---------- | --------- | --------- | ------------ | ------- | ----- | --------- | -------- | --- | --------- | --- | --- |
|                    |     |            |           |           |              | John    | Wang  | and Edwin | Olson    |     |           |     |     |
| Abstract—AprilTags |     |            | and other | passive   | fiducial     | markers | re-   |           |          |     |           |     |     |
| quire specialized  |     | algorithms |           | to detect | markers      | among   | other |           |          |     |           |     |     |
featuresinanaturalscene.Thevisionprocessingstepsgenerally
| dominate          | the computation |          | time       | of a            | tag detection  | pipeline,     | so         |     |     |     |     |     |     |
| ----------------- | --------------- | -------- | ---------- | --------------- | -------------- | ------------- | ---------- | --- | --- | --- | --- | --- | --- |
| even small        | improvements    |          | in         | marker          | detection      | can translate | to         |     |     |     |     |     |     |
| a faster          | tag detection   | system.  |            | We incorporated |                | lessons       | learned    |     |     |     |     |     |     |
| from implementing |                 | and      | supporting |                 | the AprilTag   | system        | into       |     |     |     |     |     |     |
| this improved     | system.         |          |            |                 |                |               |            |     |     |     |     |     |     |
| This              | work describes  |          | AprilTag   | 2,              | a completely   | redesigned    |            |     |     |     |     |     |     |
| tag detector      | that            | improves | robustness |                 | and efficiency | compared      |            |     |     |     |     |     |     |
| to the original   |                 | AprilTag | system.    | The             | tag            | coding scheme | is         |     |     |     |     |     |     |
| unchanged,        | retaining       |          | the same   | robustness      |                | to false      | positives  |     |     |     |     |     |     |
| inherent          | to the          | coding   | system.    | The             | new            | detector      | improves   |     |     |     |     |     |     |
| performance       | with            | higher   | detection  | rates,          | fewer          | false         | positives, |     |     |     |     |     |     |
andlowercomputationaltime.Improvedperformanceonsmall
| images allows |       | the use      | of decimated |     | input images, | resulting | in  |      |                 |     |           |                    |           |
| ------------- | ----- | ------------ | ------------ | --- | ------------- | --------- | --- | ---- | --------------- | --- | --------- | ------------------ | --------- |
| dramatic      | gains | in detection | speed.       |     |               |           |     |      |                 |     |           |                    |           |
|               |       | I.           | INTRODUCTION |     |               |           |     |      |                 |     |           |                    |           |
|               |       |              |              |     |               |           |     | Fig. | 1: Applications |     | and users | of AprilTags. From | top left, |
Fiducials are artificial visual features designed for au- clockwise: robot-to-robot localization and identification on
tomatic detection, and often carry a unique payload to MAGIC robots, object localization for Boston Dynamics’
make them distinguishable from each other. Although these Atlas robot, testing of virtual reality headsets at Valve, and
types of fiducials were first developed and popularized by trackingindividualantstostudytheirsocialorganization[4].
| augmented | reality | applications |     | [1], | [2], they | have since | been |     |     |     |     |     |     |
| --------- | ------- | ------------ | --- | ---- | --------- | ---------- | ---- | --- | --- | --- | --- | --- | --- |
widelyadoptedbytheroboticscommunity.Theirusesrange
fromgroundtruthingtoobjectdetectionandtracking,where tags with decode errors. In these cases, features such as
theycanbeusedasasimplifyingassumptioninlieuofmore support for recovering partially-occluded tag borders are
sophisticated perception. seldom useful. This functionality must be weighed against
A few key properties of fiducials make them useful for the costs of additional computation time and an increased
|                 |     |           |          |     |             |              |     | false | positive | rate. |     |     |     |
| --------------- | --- | --------- | -------- | --- | ----------- | ------------ | --- | ----- | -------- | ----- | --- | --- | --- |
| pose estimation |     | or object | tracking |     | in robotics | applications |     |       |          |       |     |     |     |
(Figure1).Theiruniquenessandhighdetectionrateareideal This work describes a method for improving AprilTag
for testing SLAM systems. Fixed fiducial markers can be detectionspeedandsensitivitywhiletradingofftheabilityto
used for visual localization or as a ground truth estimate of detect partially-occluded tags. We show that this method is
robot motion. Fiducials mounted on objects can be used to faster than the previous detection method, reducing the rate
|          |              |         |     |              |     |     |     | of  | false positives | without | sacrificing | localization | accuracy. |
| -------- | ------------ | ------- | --- | ------------ | --- | --- | --- | --- | --------------- | ------- | ----------- | ------------ | --------- |
| identify | and localize | objects |     | of interest. |     |     |     |     |                 |         |             |              |           |
ThisworkisbasedontheearlierAprilTagsystem[3].The The contributions of this paper are:
design of AprilTags as a black-and-white square tag with an anAprilTagdetectionalgorithmthatimprovesdetection
•
encoded binary payload is based on the earlier ARTag [2] rate for small tags, exhibits fewer false positives, and
andARToolkit[1].AprilTagintroducedanimprovedmethod reduces computation time compared to the previous
| of generating |     | binary | payloads, | guaranteeing |     | a minimum |     |     | algorithm |     |     |     |     |
| ------------- | --- | ------ | --------- | ------------ | --- | --------- | --- | --- | --------- | --- | --- | --- | --- |
Hammingdistancebetweentagsunderallpossiblerotations, • anewtagboundarysegmentationmethodthatisrespon-
making them more robust than earlier designs. The tag sible for many of the performance improvements, and
generation process, a lexicode-based process with minimum could be applied to other fiducial detectors
complexity heuristics, was empirically shown to reduce the • an evaluation of the effect of fewer tag candidates on
| false positive | rate | compared |     | to ARTag | designs | of similar | bit |     |                |       |     |     |     |
| -------------- | ---- | -------- | --- | -------- | ------- | ---------- | --- | --- | -------------- | ----- | --- | --- | --- |
|                |      |          |     |          |         |            |     |     | false positive | rates |     |     |     |
length. • anexperimentalcharacterizationofthelocalizationper-
Based on feedback from AprilTag users in the robotics formance of our detector on real and synthetic images
| community, | we  | determined |     | that most | users | do not | accept |     |     |     |     |     |     |
| ---------- | --- | ---------- | --- | --------- | ----- | ------ | ------ | --- | --- | --- | --- | --- | --- |
II. PRIORWORK
The authors are with the Computer Science and Engineering De- One of the earliest visual fiducial systems was introduced
partment at the University of Michigan in Ann Arbor, MI, USA. by ARToolkit [1], a library for augmented reality applica-
| {jnwang,ebolson}@umich.edu; |     |     |     | http://april.eecs.umich. |     |     |     |                                                       |     |     |     |     |     |
| --------------------------- | --- | --- | --- | ------------------------ | --- | --- | --- | ----------------------------------------------------- | --- | --- | --- | --- | --- |
| edu                         |     |     |     |                          |     |     |     | tions.ARToolkitintroducedtheblacksquaretagasatracking |     |     |     |     |     |

|     |     |     |     |     |     |     | Besides       | square-shaped |             | binary          | tags,        | other        | tag encoding |      |
| --- | --- | --- | --- | --- | --- | --- | ------------- | ------------- | ----------- | --------------- | ------------ | ------------ | ------------ | ---- |
|     |     |     |     |     |     |     | schemes       | have been     | proposed.   | In              | particular,  | reacTIVision |              | [7]  |
|     |     |     |     |     |     |     | uses a unique | topological   |             | tag recognition |              | system       | introduced   |      |
|     |     |     |     |     |     |     | by d-touch    | [8].          | FourierTags | [9]             | are radially | symmetric    |              | tags |
|     |     |     |     |     |     |     | designed      | to increase   | detection   |                 | range by     | degrading    | smoothly.    |      |
(a) ARToolkit (b) ARTag (c) AprilTag RUNE-Tags [10] are named after the circular dot patterns
|     |     |     |     |     |     |     | (rings of | unconnected | ellipses) |     | which | make up | the | fiducial |
| --- | --- | --- | --- | --- | --- | --- | --------- | ----------- | --------- | --- | ----- | ------- | --- | -------- |
marker.Thedotsarechosentoprovidelocalizationaccuracy
|     |     |     |     |     |     |     | at the expense | of  | computation |     | time, while | being | robust | to  |
| --- | --- | --- | --- | --- | --- | --- | -------------- | --- | ----------- | --- | ----------- | ----- | ------ | --- |
blurring,noise,andpartialocclusion.Pi-Tag[11]usescross-
ratiotorecognizemarkers,notingthatthecross-ratiooffour
pointsinalineisinvariantundercameraprojectivegeometry.
|           |            |          |        |                |           |      | ChromaTags   | [12]   | are an  | extension | of AprilTags, |     | where        | two |
| --------- | ---------- | -------- | ------ | -------------- | --------- | ---- | ------------ | ------ | ------- | --------- | ------------- | --- | ------------ | --- |
|           | (d)        | RUNE-Tag | (e)    | reacTIVision   |           |      |              |        |         |           |               |     |              |     |
|           |            |          |        |                |           |      | bicolor tags | are    | blended | in order  | to maximize   |     | the gradient |     |
|           |            |          |        |                |           |      | magnitude    | in the | CIELAB  | color     | space.        | The | colorspace   |     |
| Fig. 2: A | comparison | of       | visual | fiducial tags. | ARToolkit | tags |              |        |         |           |               |     |              |     |
allow arbitrary pixel patterns inside the black border, while conversion reduces the number of edges compared to the
ARTagsandAprilTagsuse2Dbinarycodes.RUNE-tagsand grayscale image, thus speeding detection.
| reacTIVision | markers  | are | two different | existing | approaches |     |                 |          |         |             |        |               |           |       |
| ------------ | -------- | --- | ------------- | -------- | ---------- | --- | --------------- | -------- | ------- | ----------- | ------ | ------------- | --------- | ----- |
|              |          |     |               |          |            |     |                 |          | III.    | TAGDETECTOR |        |               |           |       |
| to fiducial  | markers. |     |               |          |            |     |                 |          |         |             |        |               |           |       |
|              |          |     |               |          |            |     | Our system      | features |         | an improved |        | quad detector |           | which |
|              |          |     |               |          |            |     | finds candidate |          | tags in | a grayscale | image. | Each          | candidate |       |
marker, which has the advantage of providing a full 6- is then decoded to determine if they are valid AprilTag
DOF pose estimate from a single marker of known scale. detections. The method leads to fewer false positives than
ARToolkit distinguished tags by embedding arbitrary image the previous state of the art detector while reliably detecting
patterns inside the square, which were matched against a validunoccludedquads,contributingtoanoveralllowerfalse
| databaseofknownpatternsforidentification.Asthedatabase |          |       |     |                       |     |      | positive   | rate.   |     |     |     |     |     |     |
| ------------------------------------------------------ | -------- | ----- | --- | --------------------- | --- | ---- | ---------- | ------- | --- | --- | --- | --- | --- | --- |
| of recognized                                          | patterns | grew, | so  | did the computational |     | cost |            |         |     |     |     |     |     |     |
|                                                        |          |       |     |                       |     |      | A. Lessons | learned |     |     |     |     |     |     |
ofmatchingandthelikelihoodofconfusingdistinctpatterns.
ARTag [2] attempted to rectify the problem of inter-tag The improvements to the tag detector were inspired by
confusion by introducing a 2D binary barcode pattern. The user feedback about common use cases. We learned that in
binarybarcodeallowedbiterrorsindetectiontobecorrected. most deployments, detection of partially occluded tags is of
An improved detector algorithm used image gradients to limited utility. Occluded tags often have one or more bit
errors,andmostusersdisabledecodingoftagswithbiterrors
| detect tag | edges, | an improvement |     | over the | primitive | thresh- |     |     |     |     |     |     |     |     |
| ---------- | ------ | -------------- | --- | -------- | --------- | ------- | --- | --- | --- | --- | --- | --- | --- | --- |
olding method of ARToolkit. Surviving forks of the project due to the impact on false positive rates. No known users
include ARToolkitPlus [5] and Studierstube Tracker [6]. accept tags with more than two bit errors, which enables
AprilTag [3] built upon the advances of ARTag, introduc- a faster decode algorithm. In our experience, the increased
ing a lexicode-based system for generating tags. AprilTags detection speed is a favorable tradeoff against the ability to
guaranteeaminimumHammingdistancebetweentagsunder recover partially occluded tag borders.
allpossiblerotations,whileenforcingaminimumcomplexity
|            |           |     |         |                 |      |         | B. Adaptive | thresholding |     |     |     |     |     |     |
| ---------- | --------- | --- | ------- | --------------- | ---- | ------- | ----------- | ------------ | --- | --- | --- | --- | --- | --- |
| constraint | to reduce | the | rate of | false positives | from | arising |             |              |     |     |     |     |     |     |
in natural images. Localization accuracy was improved over The first step is to threshold the grayscale input image
ARTag’s previous state of the art. Moreover, AprilTag pro- into a black-and-white image. Some thresholding methods
vided a popular open source detector implementation which attempt to find a global threshold value for the entire image
encouraged its adoption by the academic community. [13], while others find local or adaptive thresholds [14]. We
The original AprilTag detector used image gradients to adoptanadaptivethresholdingapproach,wheretheideaisto
detect high-contrast edges. This has the advantage of being find the minimum and maximum values in a region around
| robust to | shadows | and | variations | in lighting | over | previ- | each pixel. |     |     |     |     |     |     |     |
| --------- | ------- | --- | ---------- | ----------- | ---- | ------ | ----------- | --- | --- | --- | --- | --- | --- | --- |
ous methods which used naive thresholding. Detection of Instead of computing the exact extrema (max and min
partially occluded tags was made possible by first fitting values) around every pixel, we divide the image into tiles
segments to the gradients, then searching over combinations of 4x4 pixels and compute the extrema within each tile. To
of segments that formed four-sided shapes, or quads. A prevent artifacts from arising between tile boundaries with
disadvantage of the segment-first approach is the large num- large differences in extreme values, we find the extrema in a
ber of candidate quads that are generated. Much processing neighborhood of 3x3 surrounding tiles, ensuring a minimum
time is spent attempting to decode invalid candidate quads. of one tile overlap when computing extrema for adjacent
Empirically, the AprilTag detector spends most of its time pixels. Each pixel is then assigned a value of white or
fitting lines to gradient edges, many of which will not be black,usingthemeanvalue(max+min)/2asthethreshold
part of valid tag detections. (Figure3b).Forourapplication,weonlyneedtoconsistently

(a) Original image (b) Adaptive thresholding (c) Segmentation
|     |     |     |     |     | (d) Detected | quads |     |     | (e) Tag | detections |     |     |     |
| --- | --- | --- | --- | --- | ------------ | ----- | --- | --- | ------- | ---------- | --- | --- | --- |
Fig. 3: Intermediate steps of the AprilTag detector. The input image (a) is binarized using adaptive thresholding (b). The
connected black and white regions are segmented into connected components (c). Component boundaries are segmented
using a novel algorithm, which efficiently clusters pixels which border the same black and white region. Finally, quads are
fit to each cluster of border pixels (d), poor quad fits and undecodable tags are discarded, and valid tag detections are output
(e).
differentiate the light and dark pixels which form the tag. function FINDBOUNDARIES(im,w,h)
| Regions | of the | image | with | insufficient | contrast, |     | colored in |     |               |           |            |       |            |
| ------- | ------ | ----- | ---- | ------------ | --------- | --- | ---------- | --- | ------------- | --------- | ---------- | ----- | ---------- |
|         |        |       |      |              |           |     |            |     | (cid:46) Find | connected | components | using | union-find |
gray in Figure 3b, are excluded from future processing to uf ← UnionFind(w·h)
| save computation |     | time.    |              |     |     |     |     |     |          |                                    | (x,y)                 |      |     |
| ---------------- | --- | -------- | ------------ | --- | --- | --- | --- | --- | -------- | ---------------------------------- | --------------------- | ---- | --- |
|                  |     |          |              |     |     |     |     |     | for each | pixel                              | do                    |      |     |
|                  |     |          |              |     |     |     |     |     | for      | each neighbor                      | (x(cid:48),y(cid:48)) | do   |     |
| C. Continuous    |     | boundary | segmentation |     |     |     |     |     |          |                                    |                       |      |     |
|                  |     |          |              |     |     |     |     |     |          | if im[x,y]=im[x(cid:48),y(cid:48)] |                       | then |     |
Given the binarized image, the next step is to find edges uf.union(y·w+x,y(cid:48)·w+x)
| which might | form | the | boundary | of  | a tag. A | straightforward |     |     |     |     |     |     |     |
| ----------- | ---- | --- | -------- | --- | -------- | --------------- | --- | --- | --- | --- | --- | --- | --- |
end if
| approach | is to | identify | edge | pixels | which | have an | opposite- |     | end | for |     |     |     |
| -------- | ----- | -------- | ---- | ------ | ----- | ------- | --------- | --- | --- | --- | --- | --- | --- |
coloredneighbor,thenformconnectedgroupsofedgepixels.
|          |      |          |        |      |      |           |       |     | end for        |        |       |                   |          |
| -------- | ---- | -------- | ------ | ---- | ---- | --------- | ----- | --- | -------------- | ------ | ----- | ----------------- | -------- |
| However, | this | approach | breaks | down | when | the white | space |     |                |        |       |                   |          |
|          |      |          |        |      |      |           |       |     | (cid:46) Group | pixels | which | form a continuous | boundary |
betweentagboundariesapproachesonlyasinglepixelwide,
|           |        |     |            |       |     |         |          |     | h←       | HashTable() |          |     |     |
| --------- | ------ | --- | ---------- | ----- | --- | ------- | -------- | --- | -------- | ----------- | -------- | --- | --- |
| which may | happen | for | physically | small | or  | faraway | tags. If |     |          |             |          |     |     |
|           |        |     |            |       |     |         |          |     | for each | pixel       | (x,y) do |     |     |
two tag boundaries are incorrectly merged, the tags will not for (x(cid:48),y(cid:48)) do
each neighbor
| be detected. | Our | proposed   | solution |       | is to segment |            | the edges |     |     |                                            |                 |      |     |
| ------------ | --- | ---------- | -------- | ----- | ------------- | ---------- | --------- | --- | --- | ------------------------------------------ | --------------- | ---- | --- |
|              |     |            |          |       |               |            |           |     |     | if im[x,y](cid:54)=im[x(cid:48),y(cid:48)] |                 | then |     |
| based on     | the | identities | of the   | black | and white     | components |           |     |     |                                            |                 |      |     |
|              |     |            |          |       |               |            |           |     |     | r                                          | ←uf.find(y·w+x) |      |     |
0
| from which | they | arise. |     |     |     |     |     |     |     |     | ←uf.find(y(cid:48)·w+x(cid:48)) |     |     |
| ---------- | ---- | ------ | --- | --- | --- | --- | --- | --- | --- | --- | ------------------------------- | --- | --- |
r 1
| Connected   |       | components     | of          | light     | and dark    | pixels    | are seg-   |     |     |     |                        |        |       |
| ----------- | ----- | -------------- | ----------- | --------- | ----------- | --------- | ---------- | --- | --- | --- | ---------------------- | ------ | ----- |
|             |       |                |             |           |             |           |            |     |     | id← | ConcatenateBits(Sort(r |        | ,r )) |
| mented      | using | the union-find |             | algorithm | [15]        | (Figure   | 3c),       |     |     |     |                        |        | 0 1   |
|             |       |                |             |           |             |           |            |     |     | if  | id∈/ h then            |        |       |
| which gives | each  | component      |             | a unique  | ID.         | For every | pair of    |     |     |     |                        |        |       |
|             |       |                |             |           |             |           |            |     |     |     | h[id]←                 | List() |       |
| adjacent    | black | and white      | components, |           | we identify |           | the pixels |     |     |     |                        |        |       |
|             |       |                |             |           |             |           |            |     |     | end | if                     |        |       |
on the boundaries of those two regions as a distinct cluster. p←(x+x(cid:48) ,y+y(cid:48)
)
| Thisclusteringcanbedoneefficientlybyusingahashtable, |     |     |     |     |     |     |     |     |     |     | 2   | 2   |     |
| ---------------------------------------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
h[id].append(p)
| indexing | each | cluster | by the | black | and white | components’ |     |     |     |     |     |     |     |
| -------- | ---- | ------- | ------ | ----- | --------- | ----------- | --- | --- | --- | --- | --- | --- | --- |
end if
| IDs, as           | described  | in Figure | 4.        | In the | aforementioned |     | case of  |     |         |     |     |     |     |
| ----------------- | ---------- | --------- | --------- | ------ | -------------- | --- | -------- | --- | ------- | --- | --- | --- | --- |
|                   |            |           |           |        |                |     |          |     | end     | for |     |     |     |
| a single          | pixel-wide | white     | component |        | separating     | two | distinct |     |         |     |     |     |     |
|                   |            |           |           |        |                |     |          |     | end for |     |     |     |     |
| black components, |            | we        | have      | solved | the problem    | by  | allowing |     |         |     |     |     |     |
end function
| the same   | white | pixels    | to appear | in      | both resulting |     | clusters. |      |              |     |                |                |               |
| ---------- | ----- | --------- | --------- | ------- | -------------- | --- | --------- | ---- | ------------ | --- | -------------- | -------------- | ------------- |
|            |       |           |           |         |                |     |           | Fig. | 4: Algorithm |     | for continuous | boundary       | segmentation. |
| D. Fitting | quads |           |           |         |                |     |           |      |              |     |                |                |               |
|            |       |           |           |         |                |     |           | The  | neighbors    | of  | (x,y) using    | 8-connectivity | are {(x +     |
| The next   | step  | is to fit | a quad    | to each | cluster        | of  | unordered |      |              |     |                |                |               |
1,y),(x−1,y+1),(x,y+1),(x+1,y+1)}.
| boundary | points, | partitioning |     | the | points | into four | groups |     |     |     |     |     |     |
| -------- | ------- | ------------ | --- | --- | ------ | --------- | ------ | --- | --- | --- | --- | --- | --- |

Candidatequads Falsedetections Falsepositiverate
corresponding to line segments. However, computing the
Old 51,075,971 145 0.000284%
optimal partition which minimizes total line fit error is New 13,623,725 6 0.000044%
computationally expensive. Even for an ordered list of n
TABLE I: False positive rate on the LabelMe dataset
points,thereareO(n4)possiblewaystopartitionthepoints.
(421,049 images). Both detectors used AprilTag-36h11 with
Our method computes an approximate partition by finding
up to 2 bit errors corrected, which has a theoretical false
a small number of corner points, then iterating through all
positive rate of 0.000570%. Noisy image regions were most
possible combinations of corner points.
likely to be decoded as false positives. The new detector’s
Firstthepointsaresortedbyangleinaconsistentwinding
continuousboundarysegmentationislesslikelytofitaquad
orderaroundtheircentroid.Thisorderingallowsustodefine
to noise, further reducing an already low false positive rate.
“neighboring points” as ranges of sorted points. Cumulative
first and second moment statistics are computed in a single
pass through these points, enabling the first and second
moments to be computed for any range of points in constant computationallyinexpensivemethodtorefinetheedgeusing
time. the original image.
Corner points are identified by attempting to fit a line to The idea is to use the image gradient along the edges
windows of neighboring points, and finding the peaks in the of the candidate quads to fit new edges, approximating
mean squared error function as the window is swept across the behavior of the original AprilTag detector. Along each
thepoints.Linefitsarecomputedusingprincipalcomponent edge, at evenly-spaced sample points, we sample the image
analysis (PCA) [16], in which an ellipse is fit to the sample gradient along the normal to the edge to find the location
mean and covariance. The best fit line is the eigenvector withthelargestgradient.Knowingtagsaredarkontheinside
corresponding to the first principal component. Using the and the winding order of the points in the quad, we reject
precomputed statistics, all the candidate line fits may be points whose gradient is not the expected sign (i.e. from
computed in O(n) time, where n is the number of points. noisy individual pixels). We compute a weighted average
The strongest peaks in the mean squared error are identified of the points along the normal, weighted by the gradient
as candidate corners. magnitude. The line fit along these weighted average points
Finally, we iterate through all permutations of four can- arethenusedastheedgesofthequad.Thequadcornersare
didate corners, fitting lines to each side of the candidate computed as the intersections of these lines.
quad. At this step we select the four corners which result Edge refinement is not crucial if one is only interested
in the smallest mean squared line fit errors. Prefiltering is in detecting tags, although it can help with the decoding of
performed to reject poor quad fits, such as those without at verysmalltags.However,theedgerefinementstepimproves
least four corners, whose mean squared errors are too large, localizationaccuracywhentagsareusedforposeestimation.
or whose corner angles deviate too far from 90◦.
IV. EXPERIMENTALRESULTS
The quad fitting step outputs a set of candidate quads for
A. False positive rate
decoding (Figure 3d). Note that the quad detector correctly
finds many quad-shaped structures in the environment, in- AkeyadvantageofAprilTagsisitsresiliencyagainstfalse
cludingspecularities,switches,andindividualtagpixels.The positive detections in natural scenes. The previous detector
decoding step, which compares the contents of the quad to was shown to have a lower rate of false positives than
known codewords, filters out the false quad candidates. theoreticallyexpected,largelyduetothecomplexityheuristic
in the tag generation process. We note that the number of
E. Quick decoding
false positives is not only a feature of the tag codewords
AstraightforwardapproachtodecodingtagsistoXORthe themselves, but also a function of the number of candidate
detected code (in each of its four possible rotations) with quads generated by the detector. A detector which generates
each the codes in a tag family. A tag is identified as the fewer candidate quads should be expected to generate fewer
code with the smallest Hamming distance from the detected false positives.
code.However,ifwelimitthenumberofbiterrorscorrected We ran an experiment to compare the performance of
to two bits or fewer, it is possible to enumerate all O(n2) the new detection algorithm against the previous one, using
possible codes within two bit errors of valid codes in a tag the same LabelMe [17] dataset as the previous paper. This
family. These codes can be precomputed and stored in a dataset consists of images of natural scenes, none of which
hashtable,speedingupdecodingfromO(n)comparisonsto contain AprilTags. Note that the likelihood of false positives
O(1), where n is the size of the tag family. isintentionallyincreasedbyallowingupto2biterrorstobe
corrected. The number of false positives is reduced by more
F. Edge refinement
than we would expect from the lower quad detection rate
The threshold image, while useful for segmentation and alone(TableI).Thedetectorisalsomoreselective,resulting
quadborderdetection,mayintroducenoiseintothethreshold inalowerfalsepositiverate.Ananalysisoftheimageswhich
image. For example, shadows and glare can impinge upon generated false positives shows that noisy image regions
an edge after thresholding, leading to poor localization weremorelikelytoaccidentallydecodetoavalidcodeword.
accuracy with the resulting tag. We provide an optional, The continuous boundary segmentation in the new detector

8
|     |     | Old |     |     |     |     |     |     |     |     |     |     | Old |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
100
|     | 7   | New |     |     |     |     |     |     |     |     |     |     | New |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
Old(decimate)
6
|     |                    | New(decimate) |     |     |     |     |     |     | 80              |     |     |     |     |     |
| --- | ------------------ | ------------- | --- | --- | --- | --- | --- | --- | --------------- | --- | --- | --- | --- | --- |
|     | )m(rorrenoitisop 5 |               |     |     |     |     |     |     | )%(detcetedsgat |     |     |     |     |     |
60
4
|     | 3   |     |     |     |     |     |     |     | 40  |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
2
20
1
|     | 0   |     |     |             |     |     |     |     | 0   |     |             |     |     |     |
| --- | --- | --- | --- | ----------- | --- | --- | --- | --- | --- | --- | ----------- | --- | --- | --- |
|     | 0   | 5   | 10  | 15          | 20  | 25  | 30  |     | 0   | 10  | 20          | 30  | 40  | 50  |
|     |     |     |     | distance(m) |     |     |     |     |     |     | distance(m) |     |     |     |
Fig. 5: Position error vs. distance. When decimated images Fig. 7: Percentage of tags detected vs. distance. The new
are used, localization performance is similar to the old detector is better able to detect tags which are small and/or
| detector’s | performance |     | using | full-size | images. |     |     | far away. |     |     |     |     |     |     |
| ---------- | ----------- | --- | ----- | --------- | ------- | --- | --- | --------- | --- | --- | --- | --- | --- | --- |
3.0
Old
New
2.5
Old(decimate)
)seerged(rorrenoitatneiro
|     | 2.0 | New(decimate) |     |     |     |     |     |     |     |     |     |     |     |     |
| --- | --- | ------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
1.5
|     |     |     |     |     |     |     |     |     | (a) 0.6m | (closest) |     | (b) 7.0m | (farthest) |     |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | -------- | --------- | --- | -------- | ---------- | --- |
1.0
Fig.8:AprilTagmosaicfordistancetestonrealimages.Im-
|     | 0.5 |     |     |     |     |     |     | agesweretakenwithaPointGreyChameleonat1296x964 |     |             |              |      |            |       |
| --- | --- | --- | --- | --- | --- | --- | --- | ---------------------------------------------- | --- | ----------- | ------------ | ---- | ---------- | ----- |
|     |     |     |     |     |     |     |     | pixels,                                        | and | each tag is | 0.167m wide. | This | experiment | shows |
0.0 that the tag detection and localization performance observed
|     | 0   | 10 20 | 30  | 40 50 | 60  | 70 80 | 90  |     |     |     |     |     |     |     |
| --- | --- | ----- | --- | ----- | --- | ----- | --- | --- | --- | --- | --- | --- | --- | --- |
off-axisangle(degrees) in simulatedimages translates to realimagery. (Rectification
Fig. 6: Orientation error vs. off-axis angle. Both variants of was performed before tag detection and localization.)
| the | tag detector | perform     | similarly |          | at small | angles, | but the    |     |     |     |     |     |     |     |
| --- | ------------ | ----------- | --------- | -------- | -------- | ------- | ---------- | --- | --- | --- | --- | --- | --- | --- |
| new | detector     | performance |           | does not | degrade  | as      | quickly as |     |     |     |     |     |     |     |
the old one when using decimated images. The error in estimated orientation is plotted with respect to
|     |     |     |     |     |     |     |     | the off-axis |     | angle (Figure | 6). |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | ------------ | --- | ------------- | --- | --- | --- | --- |
Bothlocalizationerrorexperimentswererepeatedwiththe
| algorithm | is  | likely | responsible | for | this increased |     | robustness, |      |        |           |               |          |       |         |
| --------- | --- | ------ | ----------- | --- | -------------- | --- | ----------- | ---- | ------ | --------- | ------------- | -------- | ----- | ------- |
|           |     |        |             |     |                |     |             | same | images | decimated | to half their | original | size. | The new |
as it is less likely to fit a candidate quad to noise. detectorvastlyoutperformstheolddetectorwhendecimated
|                 |     |          |     |     |     |     |     | images   | are | used, without | noticeably | affecting |             | localization |
| --------------- | --- | -------- | --- | --- | --- | --- | --- | -------- | --- | ------------- | ---------- | --------- | ----------- | ------------ |
| B. Localization |     | accuracy |     |     |     |     |     |          |     |               |            |           |             |              |
|                 |     |          |     |     |     |     |     | accuracy | vs. | non-decimated | images.    | This      | observation | is           |
To characterize the localization accuracy of our detector, borne out by the detection rate as the tags are moved farther
we generatedraytraced imageswith an idealpinhole camera away in the simulated images; the new detector is far more
model, where the tags’ true position and orientation were capable at detecting small tags (Figure 7). The ability to
known. A single tag with known side length was placed in decimateinputimagesisoneofthekeystothecomputational
the scene while varying the distance and orientation. efficiency of the new detector.
In the first experiment, tag positions were generated ran- Another question we seek to address is whether the
domly, constrained to be a fixed distance from the camera simulated results will translate to real world performance.
center, while the orientation of the tag was fixed parallel to To answer this question, we collected real images of a
| the | image plane. | The | error | in estimated |     | distance | is plotted |       |          |        |               |            |     |            |
| --- | ------------ | --- | ----- | ------------ | --- | -------- | ---------- | ----- | -------- | ------ | ------------- | ---------- | --- | ---------- |
|     |              |     |       |              |     |          |            | large | AprilTag | mosaic | at increasing | distances. |     | The camera |
with respect to the distance of the tag from the camera was aligned with the center tag of the mosaic, and moved
(Figure 5). perpendicularly away from the mosaic plane. The ground
In the second experiment, the tag position was fixed so truthwasmeasuredusingalasertapemeasure.Theestimated
that the camera’s optical axis passed through its center. The distancetothecentertagisplottedinFigure9.Inadditionto
tag orientation was generated randomly, constrained so that improvedlocalizationaccuracy,thenewdetectoralsodetects
itsnormalvectormakesthesameanglewiththecameraaxis. tags at the full range of distances, while the old detector

8
7
6
5
4
3
2
1
0
0 1 2 3 4 5 6 7 8
actualdistance(m)
)m(ecnatsiddetamitse
Old
New
Fig. 9: Estimated distance to the tag mosaic center, showing
the extended range and accuracy on real data.
100
80
60
40
20
0
0 1 2 3 4 5 6 7 8
distance(m)
)%(detcetedsgat
V. CONCLUSION
This paper describes a new AprilTag detection algorithm
which improves upon the previous detector, reducing the
rate of false positives, increasing the detection rate, and
reducingtheamountofcomputingtimeneededfordetection.
These improvements make robust tag detection viable on
computation-limited systems such as smartphones, and ex-
tendstheusefulnessoftagtrackinginreal-timeapplications.
A free AprilTag detector app is available in the iPhone App
Store1.Thedetectorimplementation,whichwasreleasedlast
year, is open-source and freely available on our website2.
REFERENCES
[1] H.KatoandM.Billinghurst,“Markertrackingandhmdcalibrationfor
avideo-basedaugmentedrealityconferencingsystem,”inAugmented
Reality, 1999.(IWAR’99) Proceedings. 2nd IEEE and ACM Interna-
tionalWorkshopon. IEEE,1999,pp.85–94.
[2] M.Fiala,“ARTag,afiducialmarkersystemusingdigitaltechniques,”
inComputerVisionandPatternRecognition,2005.CVPR2005.IEEE
ComputerSocietyConferenceon,vol.2. IEEE,2005,pp.590–596.
[3] E.Olson,“AprilTag:Arobustandflexiblevisualfiducialsystem,”in
Proceedings of the IEEE International Conference on Robotics and
Automation(ICRA). IEEE,May2011,pp.3400–3407.
[4] D. P. Mersch, A. Crespi, and L. Keller, “Tracking individuals shows
spatialfidelityisakeyregulatorofantsocialorganization,”Science,
vol.340,no.6136,pp.1090–1093,2013.
[5] D. Wagner and D. Schmalstieg, “ARToolKitPlus for pose tracking
on mobile devices,” in Proceedings of 12th Computer Vision Winter
Workshop,2007.
[6] ——,“Makingaugmentedrealitypracticalonmobilephones,part1,”
Old ComputerGraphicsandApplications,IEEE,vol.29,no.3,pp.12–15,
New 2009.
[7] R. Bencina, M. Kaltenbrunner, and S. Jorda, “Improved topological
fiducial tracking in the reactivision system,” in Computer Vision
and Pattern Recognition-Workshops, 2005. CVPR Workshops. IEEE
Fig.10:Percentageoftagsdetectedusingrealdata,showing ComputerSocietyConferenceon. IEEE,2005,pp.99–99.
[8] E.CostanzaandJ.Robinson,“Aregionadjacencytreeapproachtothe
the improvement in detection range.
detectionanddesignoffiducials.”Vision,VideoandGraphics(VVG),
pp.63–70,2003.
[9] A. Xu and G. Dudek, “Fourier tag: a smoothly degradable fiducial
markersystemwithconfigurablepayloadcapacity,”inComputerand
experiences a rapid fall-off in detection rate (Figure 10). RobotVision(CRV),2011CanadianConferenceon. IEEE,2011,pp.
40–47.
[10] F. Bergamasco, A. Albarelli, L. Cosmo, E. Rodola, and A. Torsello,
C. Computation time “Anaccurateandrobustartificialmarkerbasedoncycliccodes,”IEEE
Transactions on Pattern Analysis and Machine Intelligence, vol. PP,
IntheLabelMeexperiment,weloggedthedimensionsand no.99,pp.1–1,2016.
wall time needed to process each image. Both tag detectors [11] F. Bergamasco, A. Albarelli, and A. Torsello, “Pi-Tag: a fast image-
space marker design based on projective invariants,” Machine vision
were run in single-threaded mode on an Intel Xeon E5-2640 andapplications,vol.24,no.6,pp.1295–1310,2013.
2.5GHzcore. Thetimeper pixel,averagedacross allimages [12] A. Walters. (2015) ChromaTags: An accurate, robust, and fast
in the dataset, was about 0.254 microseconds per pixel for visualfiducialsystem.Accessedon2016-02-29.[Online].Available:
http://austingwalters.com/chromatags/
the new detector, compared to 0.374 microseconds per pixel
[13] N.Otsu,“Athresholdselectionmethodfromgray-levelhistograms,”
for the old. This translates into about 78 ms and 115 ms, Automatica,vol.11,no.285-296,pp.23–27,1975.
respectively, for a 640 x 480 image. (The absolute times are [14] C. Chow and T. Kaneko, “Automatic boundary detection of the left
ventriclefromcineangiograms,”Computersandbiomedicalresearch,
not meant to be representative, and are meaningful only in
vol.5,no.4,pp.388–410,1972.
relationtoeachother.Computationtimevariesbyprocessing [15] T.H.Cormen,C.E.Leiserson,R.L.Rivest,andC.Stein,Introduction
speed and the number of quads in an input image.) toAlgorithms,3rded. TheMITPress,2009.
[16] K.Pearson,“Onlinesandplanesofclosestfittosystemsofpointsin
As we showed above, using decimated images with the space,”TheLondon,Edinburgh,andDublinPhilosophicalMagazine
new detector does not significantly affect localization error. andJournalofScience,vol.2,no.11,pp.559–572,1901.
[17] B. C. Russell, A. Torralba, K. P. Murphy, and W. T. Freeman,
With decimation by a factor of 2, the new detector only
“Labelme: a database and web-based tool for image annotation,”
takes 0.072 microseconds per pixel, or about 22 ms for Internationaljournalofcomputervision,vol.77,no.1-3,pp.157–173,
a 640 x 480 image. The detector performance is good 2008.
enough to run on the relatively lower-powered iPhone and
similar smartphone processors, opening up new possibilities 1https://itunes.apple.com/us/app/id736108128
in embedding AprilTags into small-scale applications. 2http://april.eecs.umich.edu/apriltag/