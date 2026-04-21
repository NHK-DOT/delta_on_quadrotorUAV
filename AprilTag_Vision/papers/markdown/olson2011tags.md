|     |     | AprilTag: |     | A   | robust |     | and flexible |             | visual | fiducial | system |     |     |
| --- | --- | --------- | --- | --- | ------ | --- | ------------ | ----------- | ------ | -------- | ------ | --- | --- |
|     |     |           |     |     |        |     | Edwin        | Olson       |        |          |        |     |     |
|     |     |           |     |     |        |     | University   | of Michigan |        |          |        |     |     |
ebolson@umich.edu
http://april.eecs.umich.edu
| Abstract—While |     | the use | of naturally-occurring |     |     | features | is  | a   |     |     |     |     |     |
| -------------- | --- | ------- | ---------------------- | --- | --- | -------- | --- | --- | --- | --- | --- | --- | --- |
centralfocusofmachineperception,artificialfeatures(fiducials)
| play an important |     | role | in creating | controllable |     | experiments, |     |     |     |     |     |     |     |
| ----------------- | --- | ---- | ----------- | ------------ | --- | ------------ | --- | --- | --- | --- | --- | --- | --- |
groundtruthing,andinsimplifyingthedevelopmentofsystems
| where perception       | is              | not the | central  | objective. |              |            |             |     |     |     |     |     |     |
| ---------------------- | --------------- | ------- | -------- | ---------- | ------------ | ---------- | ----------- | --- | --- | --- | --- | --- | --- |
| We describe            | a new           | visual  | fiducial | system     | that         | uses       | a 2D bar    |     |     |     |     |     |     |
| code style             | “tag”, allowing |         | full     | 6 DOF      | localization |            | of features |     |     |     |     |     |     |
| from a single          | image.          | Our     | system   | improves   |              | upon       | previous    |     |     |     |     |     |     |
| systems, incorporating |                 | a       | fast and | robust     | line         | detection  | system,     |     |     |     |     |     |     |
| a stronger             | digital         | coding  | system,  | and        | greater      | robustness | to          |     |     |     |     |     |     |
occlusion,warping,andlensdistortion.Whilesimilarinconcept
| to the ARTag | system,        |     | our method |         | is fully | open | and the |     |     |     |     |     |     |
| ------------ | -------------- | --- | ---------- | ------- | -------- | ---- | ------- | --- | --- | --- | --- | --- | --- |
| algorithms   | are documented |     | in         | detail. |          |      |         |     |     |     |     |     |     |
I. INTRODUCTION
| Visual | fiducials | are | artificial | landmarks |     | designed | to be |     |     |     |     |     |     |
| ------ | --------- | --- | ---------- | --------- | --- | -------- | ----- | --- | --- | --- | --- | --- | --- |
easytorecognizeanddistinguishfromoneanother.Although Fig.1. Exampledetections.Thispaperdescribesavisualfiducialsystem
basedon2Dplanartargets.Thedetectorisrobusttolightingvariationand
related to other 2D barcode systems such as QR codes [1], occlusionsandproducesaccuratelocalizationsofthetags.
| they have     | significantly |           | goals    | and applications. |             | With | a QR   |     |     |     |     |     |     |
| ------------- | ------------- | --------- | -------- | ----------------- | ----------- | ---- | ------ | --- | --- | --- | --- | --- | --- |
| code, a human | is            | typically | involved |                   | in aligning | the  | camera |     |     |     |     |     |     |
with the tag and photographs it at fairly high resolution generate user interfaces that overlay robots’ plans and task
obtaining hundreds of bytes, such as a web address. In assignments onto a head-mounted display [6].
|             |        |          |     |         |             |     |         |     | Performance | evaluation | and benchmarking |     | of robot sys- |
| ----------- | ------ | -------- | --- | ------- | ----------- | --- | ------- | --- | ----------- | ---------- | ---------------- | --- | ------------- |
| contrast, a | visual | fiducial | has | a small | information |     | payload |     |             |            |                  |     |               |
(perhaps12bits),butisdesignedtobeautomaticallydetected temshavebecomecentralissuesfortheresearchcommunity;
andlocalizedevenwhenitisatverylowresolution,unevenly visual fiducials are particularly helpful in this domain. For
lit, oddly rotated, or tucked away in the corner of an example, fiducials can be used to generate ground-truth
otherwise cluttered image. Aiding their detection at long robot trajectories and close control loops [7]. Similarly,
|                |           |     |               |     |         |       |      | artificial |     | features can make | it possible | to  | evaluate Simulta- |
| -------------- | --------- | --- | ------------- | --- | ------- | ----- | ---- | ---------- | --- | ----------------- | ----------- | --- | ----------------- |
| ranges, visual | fiducials |     | are comprised |     | of many | fewer | data |            |     |                   |             |     |                   |
cells: the alignment markers of a QR tag comprise about neous Localization and Mapping (SLAM) algorithms under
268 pixels (not including required headers or the payload), controlled algorithms [8]. Robotics applications have led to
whereas the visual fiducials described in this paper range thedevelopmentofadditionaltagdetectionsystems[9],[10].
from about 49 to 100 pixels, including the payload. Designing robust fiducials while minimizing the size
|        |            |         |     |          |     |          |        | required |     | is a challenging | both | from a | marker detection |
| ------ | ---------- | ------- | --- | -------- | --- | -------- | ------ | -------- | --- | ---------------- | ---- | ------ | ---------------- |
| Unlike | 2D barcode | systems |     | in which | the | position | of the |          |     |                  |      |        |                  |
barcode in the image is unimportant, visual fiducial systems standpoint (which pixels in the image correspond to a tag?)
provide camera-relative position and orientation of a tag. and from a error-tolerant data coding standpoint (which tag
| Fiducialsystemsalsoaredesignedtodetectmultiplemarkers |     |     |     |     |     |     |     | is  | it?) |     |     |     |     |
| ----------------------------------------------------- | --- | --- | --- | --- | --- | --- | --- | --- | ---- | --- | --- | --- | --- |
in a single image. Inthispaper,wedescribeanewvisualfiducialsystemthat
|                 |     |         |     |         |      |       |           | significantly |     | improves | performance | over | previous systems. |
| --------------- | --- | ------- | --- | ------- | ---- | ----- | --------- | ------------- | --- | -------- | ----------- | ---- | ----------------- |
| Visual fiducial |     | systems | are | perhaps | best | known | for their |               |     |          |             |      |                   |
application to augmented reality, which spurred the devel- The central contributions of this paper are:
opment of several popular systems including ARToolkit [2] Wedescribeamethodforrobustlydetectingvisualfidu-
•
and ARTag [3]. Real-world objects can be augmented with cials. We propose a graph-based image segmentation
visual fiducials, allowing virtually-generated imagery to be algorithmbasedonlocalgradientsthatallowslinestobe
super-imposed. Similarly, visual fiducials can be used for precisely estimated. We also describe a quad extraction
basic motion capture [4]. method that can handle significant occlusions.
Visual fiducial systems have been used to improve hu- We demonstrate that our detection system provides
•
man/robot interaction, allowing humans to signal commands significantly better localization accuracy than previous
| (such as “follow |     | me” or | “wait | here”) | by flashing |     | an appro- |     | systems. |     |     |     |     |
| ---------------- | --- | ------ | ----- | ------ | ----------- | --- | --------- | --- | -------- | --- | --- | --- | --- |
priate card to a robot [5]. Planar tags have also been used to • Wedescribeanewcodingsystemthataddressproblems

uniqueto2Dbarcodingsystems:robustnesstorotation, finally Studierstube Tracker [12]. These versions introduced
and robustness to false positives arising from natural digitally-encodedpayloadslikethoseusedinARTag.Despite
imagery. As demonstrated by our experimental results, being later work, our experiments show that these encoding
our coding system provides significant theoretical and systemsdonotperformaswellasthatusedbyARTag,which
real-world benefits over previous work. in turn is outperformed by our coding system.
• We specify and provide results on a set of benchmarks In addition to monochrome tags, other coding systems
whichwillallowbettercomparisonsoffiducialsystems have been developed. For example, color information has
in the future. been used to increase the amount of information that can be
In contrast to previous methods (including ARTag and encoded[13],[14].Tagsusingretro-reflectors[15]havealso
|              |           |     |                    |     |     |          |       | been used. | A particularly | interesting | approach | is that used |
| ------------ | --------- | --- | ------------------ | --- | --- | -------- | ----- | ---------- | -------------- | ----------- | -------- | ------------ |
| Studierstube | Tracker), |     | our implementation |     | is  | released | under |            |                |             |          |              |
an Open Source license, and its algorithms and implementa- by Bokode [16], which exploits the bokeh effect to detect
tion are well documented. The closed nature of these other extremelysmalltagsbyintentionallydefocusingthecamera.
| systems            | was a       | challenge   | for         | our experimental |          | evaluation.   |           |     |     |     |     |     |
| ------------------ | ----------- | ----------- | ----------- | ---------------- | -------- | ------------- | --------- | --- | --- | --- | --- | --- |
| For the            | comparisons | in          | this paper, | we               | have     | used the      | limited   |     |     |     |     |     |
| publicly-available |             | information |             | to enable        | as       | many          | objective |     |     |     |     |     |
| comparisons        | as          | possible.   | On          | the other        | hand,    | ARToolkitPlus |           |     |     |     |     |     |
| is open            | source and  | so          | we were     | able to          | make     | a more        | detailed  |     |     |     |     |     |
| comparison.        | In          | addition    | to code,    | we               | are also | making        | our       |     |     |     |     |     |
evaluationcodeavailableinordertomakeiteasierforfuture
| authors       | to perform    | comparisons. |              |               |            |         |          |     |     |     |     |     |
| ------------- | ------------- | ------------ | ------------ | ------------- | ---------- | ------- | -------- | --- | --- | --- | --- | --- |
| In the        | next section, |              | we review    | related       | work.      | We      | describe |     |     |     |     |     |
| our method    | in the        | following    |              | two sections: |            | the tag | detector |     |     |     |     |     |
| in Section    | 3, and        | the coding   | system       |               | in Section | 4. In   | Section  |     |     |     |     |     |
| 5, we provide | an            | experimental |              | evaluation    | of         | our     | methods, |     |     |     |     |     |
| comparing     | them          | to previous  | algorithms.  |               |            |         |          |     |     |     |     |     |
|               |               | II.          | PREVIOUSWORK |               |            |         |          |     |     |     |     |     |
ARToolkit [11] was among the first tag tracking systems, Fig. 2. Input image. This paper describe the processing of this sample
|     |     |     |     |     |     |     |     | image which | contains two | tags. This | example is purposefully | simple for |
| --- | --- | --- | --- | --- | --- | --- | --- | ----------- | ------------ | ---------- | ----------------------- | ---------- |
and was targeted at artificial reality applications. Like the explanatory reasons, though note that the tags are not rigidly planar. See
systemsthatwouldfollow,itstagscontainedasquare-shaped Fig.1foramorechallengingresult.
payloadsurroundedbyablackborder.Itdiffered,however,in
thatitspayloadwasnotdirectlyencodedinbinary:instead,it Besides two-dimensional barcodes, a number of other
usedsymbolssuchasthelatincharacter’A’.Whendecoding artificial landmarks have been developed. Overhead cameras
|     |     |     |     |     |     |     |     | have been | used to track | robots | equipped | with blinking |
| --- | --- | --- | --- | --- | --- | --- | --- | --------- | ------------- | ------ | -------- | ------------- |
atag,thepayloadofthetag(sampledathighresolution)was
correlated against a database of known tags, with the best- LEDs [17]. In contrast, the NorthStar system puts the vi-
correlating tag reported to the user. A major disadvantage sual fiducials on the ceiling [18]. Two-dimensional planar
|         |          |        |               |     |      |            |      | systems, | like the one | described | in this paper, | have two |
| ------- | -------- | ------ | ------------- | --- | ---- | ---------- | ---- | -------- | ------------ | --------- | -------------- | -------- |
| of this | approach | is the | computational |     | cost | associated | with |          |              |           |                |          |
decoding tags, since each template required a separate, mainadvantagesoverLED-basedsystems:thetargetscanbe
slow correlation operation. A second disadvantage is that cheaplyprintedonastandardprinter,andtheyprovide6DOF
it is difficult to generate templates that are approximately position estimation without the need for multiple LEDs.
| orthogonal | to each   | other. |      |     |           |     |          |     |      |          |     |     |
| ---------- | --------- | ------ | ---- | --- | --------- | --- | -------- | --- | ---- | -------- | --- | --- |
|            |           |        |      |     |           |     |          |     | III. | DETECTOR |     |     |
| The tag    | detection | scheme | used | by  | ARToolkit | is  | based on |     |      |          |     |     |
a simple binarization of the input image based on a user- Oursystemiscomposedoftwomajorcomponents:thetag
specified threshold. This scheme is very fast, but not robust detector and the coding system. In this section, we describe
tochangesinillumination.Ingeneral,ARToolkit’sdetections thedetectorwhosejobistoestimatethepositionofpossible
can not handle even modest occlusions of the tag’s border. tags in an image. Loosely speaking, the detector attempts to
ARTag [3] provided improved detection and coding find four-sided regions (“quads”) that have a darker interior
schemes. Like our own approach, the detection mechanism thantheirexterior.Thetagsthemselveshaveblackandwhite
wasbasedontheimagegradient,makingitrobusttochanges borders in order to facilitate this (see Fig. 2).
inlighting.Whilethedetailsofthedetectoralgorithmarenot The detection process is comprised of several distinct
public, ARTag’s detection mechanism is able to detect tags phases,whicharedescribedinthefollowingsubsectionsand
whoseborderispartiallyoccluded.ARTagalsoprovidedthe illustrated using the example shown in Fig. 2.
firstcodingsystembasedonforwarderrorcorrection,which Note that the quad detector is designed to have a very
madetagseasiertogenerate,fastertocorrelate,andprovided low false negative rate, and consequently has a high false
greater orthogonality between tags. positive rate. We rely on the coding system (described in
TheperformanceofARTaginspiredseveralimprovements the next section) to reduce this false positive rate to useful
| to ARToolkit, | which |     | evolved | into ARToolkitPlus |     |     | [2], and | levels. |     |     |     |     |
| ------------- | ----- | --- | ------- | ------------------ | --- | --- | -------- | ------- | --- | --- | --- | --- |

Fig.3. Earlyprocessingsteps.Thetagdetectionalgorithmbeginsbycomputingthegradientateverypixel,computingtheirmagnitudes(first)anddirection
(second).Usingagraph-basedmethod,pixelswithsimilargradientdirectionsandmagnitudeareclusteredintocomponents(third).Usingweightedleast
squares,alinesegmentisthenfittothepixelsineachcomponent(fourth).Thedirectionofthelinesegmentisdeterminedbythegradientdirection,so
thatsegmentsaredarkontheleft,lightontheright.Thedirectionofthelinesarevisualizedbyshortperpendicular“notches”attheirmidpoint;notethat
these“notches”alwayspointtowardsthelighterregion.
| A. Detecting | line    | segments  |           |       |           |            |           |     |     |     |     |     |     |     |
| ------------ | ------- | --------- | --------- | ----- | --------- | ---------- | --------- | --- | --- | --- | --- | --- | --- | --- |
| Our approach |         | begins by | detecting | lines | in        | the image. | Our       |     |     |     |     |     |     |     |
| approach,    | similar | in basic  | approach  | to    | the ARTag |            | detector, |     |     |     |     |     |     |     |
computesthegradientdirectionandmagnitudeateverypixel
| (see Fig. | 3) and | agglomeratively |     | clusters |     | the pixels | into |     |     |     |     |     |     |     |
| --------- | ------ | --------------- | --- | -------- | --- | ---------- | ---- | --- | --- | --- | --- | --- | --- | --- |
componentswithsimilargradientdirectionsandmagnitudes.
| The clustering |              | algorithm | is     | similar | to the     | graph-based |         |     |     |     |     |     |     |     |
| -------------- | ------------ | --------- | ------ | ------- | ---------- | ----------- | ------- | --- | --- | --- | --- | --- | --- | --- |
| method of      | Felzenszwalb |           | [19]:  | a graph | is created | in          | which   |     |     |     |     |     |     |     |
| each node      | represents   | a         | pixel. | Edges   | are        | added       | between |     |     |     |     |     |     |     |
| adjacent       | pixels with  | an edge   | weight | equal   | to         | the pixels’ | dif-    |     |     |     |     |     |     |     |
ferenceingradientdirection.Theseedgesarethensortedand
processedintermsofincreasingedgeweight:foreachedge,
| we test | whether | the connected |     | components |     | that the | pixels |     |     |     |     |     |     |     |
| ------- | ------- | ------------- | --- | ---------- | --- | -------- | ------ | --- | --- | --- | --- | --- | --- | --- |
belong to should be joined together. Given a component n, Fig.4. Quaddetectionandsampling.Fourquadsaredetectedintheimage
we denote the range of gradient directions as D(n) and the (whichcontainstwotags).Thethirddetectedquadcorrespondstothreeof
range of magnitudes as M(n). Put another way, and the edges of the foreground tag plus the edge of the paper (See Fig. 2).
D(n)
|     |     |     |     |     |     |     |     | A fourth | quad is | detected | around one | of the payload | bits | of the larger |
| --- | --- | --- | --- | --- | --- | --- | --- | -------- | ------- | -------- | ---------- | -------------- | ---- | ------------- |
M(n) are scalar values representing the difference between tag.Thesetwoextraneousdetectionsareeventuallydiscardedbecausetheir
the maximum and minimum values of the gradient direction payload is invalid. The white dots correspond to samples around the tags
|               |               |     |     |          |     |           |      | borderwhich | are | used to | fit a linear | model of intensity | of “white” | pixels; |
| ------------- | ------------- | --- | --- | -------- | --- | --------- | ---- | ----------- | --- | ------- | ------------ | ------------------ | ---------- | ------- |
| and magnitude | respectively. |     | In  | the case | of  | D(), some | care |             |     |         |              |                    |            |         |
amodelissimilarlyfitfortheblackpixels.Thesetwomodelsareusedto
must be taken to handle 2π wrap-around. However, since thresholdthedatapayloadbits,shownasyellowdots.
| useful edges             | will          | have a | span of | much           | less       | than π | degrees, |           |       |               |          |     |            |            |
| ------------------------ | ------------- | ------ | ------- | -------------- | ---------- | ------ | -------- | --------- | ----- | ------------- | -------- | --- | ---------- | ---------- |
| this is straightforward. |               |        | Given   | two components |            | n      | and m,   |           |       |               |          |     |            |            |
| we join                  | them together | if     | both    | of the         | conditions | below  | are      |           |       |               |          |     |            |            |
|                          |               |        |         |                |            |        |          | be sorted | using | a linear-time | counting |     | sort [20]. | The actual |
satisfied: mergingoperationcanbeefficientlycarriedoutbytheunion-
|        |     |                    |     |     |     |          | (1) | find algorithm |           | [20] | with the  | upper and | lower       | bounds of |
| ------ | --- | ------------------ | --- | --- | --- | -------- | --- | -------------- | --------- | ---- | --------- | --------- | ----------- | --------- |
| D(n∪m) |     | ≤ min(D(n),D(m))+K |     |     |     | D /|n∪m| |     |                |           |      |           |           |             |           |
|        |     |                    |     |     |     |          |     | gradient       | direction | and  | magnitude | stored    | in a simple | array     |
M(n∪m) ≤ min(M(n),M(m))+K M /|n∪m| indexed by each component’s representative member.
The conditions are adapted from [19] and can be intu- Thisgradient-basedclusteringmethodissensitivetonoise
itively understood: small values of D() and M() indicate in the image: even modest amounts of noise will cause local
components with little intra-component variation. Two clus- gradientdirectionstovary,inhibitingthegrowthofthecom-
ponents.Thesolutiontothisproblemistolow-passfilterthe
| ters are | joined together |     | if their | union | is about | as  | uniform |     |     |     |     |     |     |     |
| -------- | --------------- | --- | -------- | ----- | -------- | --- | ------- | --- | --- | --- | --- | --- | --- | --- |
as the clusters taken individually. A modest increase in image [19], [21]. Unlike other problem domains where this
intra-component variation is permitted via the K and K filtering can blur useful information in the image, the edges
|             |         |      |         |         |     | D              | M   |          |                   |     |             |          |               |     |
| ----------- | ------- | ---- | ------- | ------- | --- | -------------- | --- | -------- | ----------------- | --- | ----------- | -------- | ------------- | --- |
|             |         |      |         |         |     |                |     | of a tag | are intrinsically |     | large-scale | features | (particularly | in  |
| parameters, | however | this | rapidly | shrinks | as  | the components |     |          |                   |     |             |          |               |     |
become larger. During early iterations, the K parameters comparison to the data field), and so this filtering does not
|             |       |                |     |            |     |                   |     | cause information |     | loss. | We recommend |     | a value of | σ =0.8. |
| ----------- | ----- | -------------- | --- | ---------- | --- | ----------------- | --- | ----------------- | --- | ----- | ------------ | --- | ---------- | ------- |
| essentially | allow | each component |     | to “learn” |     | its intra-cluster |     |                   |     |       |              |     |            |         |
variation. In our experiments, we used K = 100 and Once the clustering operation is complete, line segments
D
K =1200, though the algorithm works well over a broad are fit to each connected component using a traditional
M
range of values. least-squaresprocedure,weightingeachpointbyitsgradient
For performance reasons, the edge weights are quantized magnitude (see Fig. 3). We adjust each line segment so that
and stored as fixed-point numbers. This allows the edges to the dark side of the line is on its left, and the light side is

on its right. In the next phase of processing, this allows us matrix are typically 4 × 4, but every position on the tag
to enforce a winding rule around each quad. is at z = 0 in the tag’s coordinate system. Thus, we can
The segmentation algorithm is the slowest phase in our rewrite every tag coordinate as a 2D homogeneous point
detection scheme. As an option, this segmentation can be with z implicitly zero, and remove the third column of the
performed at half the image resolution with a 4x improve- extrinsics matrix, forming the truncated extrinsics matrix.
mentinspeed.Thesub-samplingoperationcanbeefficiently We represent the rotation components of P as R and the
ij
combined with the recommended low-pass filter. The conse- translationcomponentsasT .Wealsorepresenttheunknown
k
quence of this optimization is a modestly reduced detection scale factor as s:
range, since very small quads may no longer be detected.
h00 h01 h02
 
B. Quad detection
h10 h11 h12 =sPE
 h20 h21 h22 
At this point, a set of directed line segments have been
c o c o h f m a l l i l n p e e u n t g s e e e d g i m f s o e r t n o a ts n do t i h m a th t a i g f s o e w r . m T hi h a le e 4 b n -s e e i i x d n t e g d ta a s s s k ha r i o p s b e u , to s i. t e fi a . n , s d a p q s o e u s q a s u i d b e . l n e T c h e to e s =s   f 0 0 x f 0 0 y 1 0 0 0 0 0      R R R 0 1 2 0 0 0 R R R 0 1 2 1 1 1 T T T x y z    (2)
 0 0 1 
occlusions and noise in the line segmentations.
NotethatwecannotdirectlysolveforE becauseP isrank
Our approach is based on a recursive depth-first search
deficient. We can expand the right hand side of Eqn. 2, and
with a depth of four: each level of the search tree adds an
write the expression for each h as a set of simultaneous
edgetothequad.Atdepthone,weconsideralllinesegments. ij
equations:
At depths two through four, we consider all of the line
segments that begin “close enough” to where the previous
h00 = sR00f
x
(3)
line segment ended and which obey a counter-clockwise
winding order. Robustness to occlusions and segmentation
h01 = sR01f
x
errors is handled by adjusting the “close enough” threshold: h02 = sT x f x
by making the threshold large, significant gaps around the ...
edges can be handled. Our threshold for “close enough” is
ThesearealleasilysolvedfortheelementsofR andT
twice the length of the line plus five additional pixels. This ij k
except for the unknown scale factor s. However, since the
is a large threshold which leads to a low false negative rate,
columns of a rotation matrix must all be of unit magnitude,
but also results in a high false positive rate.
we can constrain the magnitude of s. We have two columns
We populate a two-dimensional lookup table to accelerate
oftherotationmatrix,sowecomputesasthegeometricthe
queries for line segments that begin near a point in space.
geometric average of their magnitudes. The sign of s can
With this optimization, along with early rejection of can-
be recovered by requiring that the tag appear in front of the
didate quads that do not obey the winding rule, or which
camera, i.e., that T < 0. The third column of the rotation
use a segment more than once, the quad detection algorithm z
matrix can be recovered by computing the cross product of
representsasmallfractionofthetotalcomputationalrequire-
the two known columns, since the columns of a rotation
ments.
matrix must be orthonormal.
Once four lines have been found, a candidate quad detec-
The DLT procedure and the normalization procedure
tioniscreated.Thecornersofthisquadaretheintersections
above do not guarantee that the rotation matrix is strictly
of the lines that comprise it. Because the lines are fit using
orthonormal. To correct this, we compute the polar decom-
data from many pixels, these corner estimates are accurate
position of R, which yields a proper rotation matrix while
to a small fraction of a pixel.
minimizing the Frobenius matrix norm of the error [23].
C. Homography and extrinsics estimation
IV. PAYLOADDECODING
Wecomputethe3×3homographymatrixthatprojects2D
The final task is to read the bits from the payload field.
pointsinhomogeneouscoordinatesfromthetag’scoordinate
Wedothisbycomputingthetag-relativecoordinatesofeach
system (in which [0 0 1]T is at the center of the tag and the
bitfield,transformingthemintoimagecoordinatesusingthe
tag extends one unit in the xˆ and yˆ directions) to the 2D
homography, and then thresholding the resulting pixels. In
image coordinate system. The homography is computed us-
order to be robust to lighting (which can vary not only from
ing the Direct Linear Transform (DLT) algorithm [22]. Note
tag to tag, but also within a tag), we use a spatially-varying
that since the homography projects points in homogeneous
threshold.
coordinates, it is defined only up to scale.
Specifically,webuildspatially-varyingmodeloftheinten-
Computation of the tag’s position and orientation requires
sityof“black”pixels,andasecondmodelfortheintensityof
additional information: the camera’s focal length and the
“white”models.Weusetheborderofthetag,whichcontains
physical size of the tag. The 3 × 3 homography matrix
knownexamplesofbothwhiteandblackpixels,tolearnthis
(computed by the DLT) can be written as the product of
model (see Fig. 4). We use the following intensity model:
the 3×4 camera projection matrix P (which we assume is
known)andthe4×3truncatedextrinsicsmatrixE.Extrinsics I(x,y)=Ax+Bxy+Cy+D (4)

Thismodelhasfourparameterswhichareeasilycomputed The ARTag encoding system, for example, explicitly forbids
using least squares regression. We build two such models, two codes because they are too likely to occur by chance.
one for black, the other for white. The threshold used when Rather than identify problematic tags manually, we fur-
decoding data bits is then just the average of the predicted ther modify the lexicode generation algorithm by rejecting
intensity values of the black and white models. candidatecodewordsthatresultinsimplegeometricpatterns.
|     |     |     |     |     |     |     |     | Our metric | is based | on  | the number |     | of rectangles | required | to  |
| --- | --- | --- | --- | --- | --- | --- | --- | ---------- | -------- | --- | ---------- | --- | ------------- | -------- | --- |
V. CODINGSYSTEM generate the tag’s 2D pattern. For example, a solid pattern
|      |          |         |            |     |        |       |           | requires | just one | rectangle, |     | while | a black-white-black |     | stripe |
| ---- | -------- | ------- | ---------- | --- | ------ | ----- | --------- | -------- | -------- | ---------- | --- | ----- | ------------------- | --- | ------ |
| Once | the data | payload | is decoded |     | from a | quad, | it is the |          |          |            |     |       |                     |     |        |
job of the coding system to determine it is valid or not. The wouldrequiretworectangles(onelargeblackrectanglewith
|            |          |        |                    |        |          |       |          | a smaller | white    | rectangle    | drawn           |         | second). | Our hypothesis, |           |
| ---------- | -------- | ------ | ------------------ | ------ | -------- | ----- | -------- | --------- | -------- | ------------ | --------------- | ------- | -------- | --------------- | --------- |
| goals of   | a coding | system | are to:            |        |          |       |          |           |          |              |                 |         |          |                 |           |
|            |          |        |                    |        |          |       |          | supported | by       | experimental |                 | results | later    | in this         | paper, is |
| • Maximize | the      | number | of distinguishable |        |          | codes |          |           |          |              |                 |         |          |                 |           |
|            |          |        |                    |        |          |       |          | that tag  | patterns | with         | high complexity |         | (which   | require         | many      |
| • Maximize | the      | number | of bit             | errors | that can | be    | detected |           |          |              |                 |         |          |                 |           |
rectanglestobereconstructed)occurlessfrequentlyinnature
or corrected
|            |        |       |                    |         |           |      |          | and thus       | lead       | to lower  | false  | positive      | rates. |              |            |
| ---------- | ------ | ----- | ------------------ | ------- | --------- | ---- | -------- | -------------- | ---------- | --------- | ------ | ------------- | ------ | ------------ | ---------- |
| • Minimize | the    | false | positive/inter-tag |         | confusion |      | rate     |                |            |           |        |               |        |              |            |
|            |        |       |                    |         |           |      |          | Using          | this idea, | we        | again  | modify        | the    | lexicode     | generation |
| • Minimize | the    | total | number             | of bits | per tag   | (and | thus the |                |            |           |        |               |        |              |            |
|            |        |       |                    |         |           |      |          | algorithm      | to reject  | candidate |        | codewords     |        | that are too | simple.    |
| size       | of the | tag)  |                    |         |           |      |          |                |            |           |        |               |        |              |            |
|            |        |       |                    |         |           |      |          | We approximate |            | the       | number | of rectangles |        | required     | to gen-    |
These goals are often in conflict, and so a given code erate the tag’s pattern with a simple greedy approach that
| represents | a trade-off. |     | In this | section, | we describe |     | a new |     |     |     |     |     |     |     |     |
| ---------- | ------------ | --- | ------- | -------- | ----------- | --- | ----- | --- | --- | --- | --- | --- | --- | --- | --- |
repeatedlyconsidersallpossiblerectanglesandaddstheone
| coding system | based |          | on lexicodes | that | provides  | significant |      |              |      |             |           |        |               |          |           |
| ------------- | ----- | -------- | ------------ | ---- | --------- | ----------- | ---- | ------------ | ---- | ----------- | --------- | ------ | ------------- | -------- | --------- |
|               |       |          |              |      |           |             |      | that reduces | the  | error       | the most. | Since  | the           | tags are | generally |
| advantages    | over  | previous | methods.     | Our  | procedure | can         | gen- |              |      |             |           |        |               |          |           |
|               |       |          |              |      |           |             |      | very small,  | this | computation |           | is not | a bottleneck. |          | Tags with |
eratelexicodeswithavarietyofproperties,allowingtheuser
|     |     |     |     |     |     |     |     | a minimum | complexity |     | less | than | a threshold | (typically | 10  |
| --- | --- | --- | --- | --- | --- | --- | --- | --------- | ---------- | --- | ---- | ---- | ----------- | ---------- | --- |
to use a code that best fits their needs. in our experiments) are rejected. The appropriateness and
|     |     |     |     |     |     |     |     | effectiveness | of  | this heuristic |     | are demonstrated |     | in  | the results |
| --- | --- | --- | --- | --- | --- | --- | --- | ------------- | --- | -------------- | --- | ---------------- | --- | --- | ----------- |
A. Methodology
section.
We propose the use of a modified lexicode [24]. Classical Lastly, we have empirically observed lower false positive
lexicodes are parameterized by two quantities: the number scores by making one more modification to the lexicode
of bits n in each codeword and the minimum Hamming generation algorithm. Rather than test codewords in order
| distance | between | any | two codewords |     | d.  | Lexicodes | can |           |          |             |     |           |       |       |            |
| -------- | ------- | --- | ------------- | --- | --- | --------- | --- | --------- | -------- | ----------- | --- | --------- | ----- | ----- | ---------- |
|          |         |     |               |     |     |           |     | (0, 1, 2, | 3, ...), | we consider |     | (b, b+1p, | b+2p, | b+3p, | ...) where |
correct ⌊(d−1)/2⌋ bit errors and detect d/2 bit errors. b is an arbitrary number, p is a large prime, and the lowest
For convenience, we will denote a 36 bit encoding with a bits are kept at each step. Intuitively, the tags generated
n
minimumHammingdistanceof10(forexample)asa36h10 by this method have greater entropy at every bit position;
code. the lexicographic order, on the other hand, favors small-
| Lexicodes | derive | their | name | from | the heuristic |     | used to |               |     |                  |     |     |             |     |            |
| --------- | ------ | ----- | ---- | ---- | ------------- | --- | ------- | ------------- | --- | ---------------- | --- | --- | ----------- | --- | ---------- |
|           |        |       |      |      |               |     |         | valued codes. |     | The disadvantage |     | of  | this method | is  | that fewer |
generate valid codewords: candidate codewords are consid- distinguishable codes are created: the lexicographic ordering
eredinlexicographicorder(fromsmallesttolargest),adding tends to pack codewords quite densely, whereas the more
new codewords to the codebook when they are at least a randomorderresultsinalessefficientpackingofcodewords.
distance d from every codeword previously added to the Tosummarize,weusealexicodesystemthatcangenerate
codebook.Whileverysimple,thisschemeisoftenveryclose
|     |     |     |     |     |     |     |     | codes for | any | arbitrary | tag | size (e.g., | 3x3, | 4x4, | 5x5, 6x6) |
| --- | --- | --- | --- | --- | --- | --- | --- | --------- | --- | --------- | --- | ----------- | ---- | ---- | --------- |
to optimal [25]. and minimum Hamming distance. Our approach explicitly
In the case of visual fiducials, the coding scheme must be guarantees the minimum Hamming distance for all four
robust to rotation. In other words, it is critical that when rotations of each tag and eliminates tags which are of
a tag is rotated by 90, 180, or 270 degrees, that it still low geometric complexity. Computing the tags can be an
| have a Hamming |     | distance | of d | from | every other | code. | The |           |            |     |        |      |          |            |       |
| -------------- | --- | -------- | ---- | ---- | ----------- | ----- | --- | --------- | ---------- | --- | ------ | ---- | -------- | ---------- | ----- |
|                |     |          |      |      |             |       |     | expensive | operation, |     | but is | done | offline. | Small tags | (5x5) |
standard lexicode generation algorithm does not guarantee can be easily computed in seconds or minutes, but larger
this property. However, the standard generation algorithm tags (6x6) can take several days of CPU time. Many useful
can be trivially extended to support this: when testing a code families are already computed and distributed with our
new candidate codeword, we can simply ensure that all four software;mostuserswillnotneedtogeneratetheirowncode
| rotations | have the | required | minimum |     | Hamming | distance. |     |     |     |     |     |     |     |     |     |
| --------- | -------- | -------- | ------- | --- | ------- | --------- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
families.
| The fact       | that the   | lexicode | algorithm   |     | can be easily   | extended |        |          |            |          |     |     |     |     |     |
| -------------- | ---------- | -------- | ----------- | --- | --------------- | -------- | ------ | -------- | ---------- | -------- | --- | --- | --- | --- | --- |
|                |            |          |             |     |                 |          |        | B. Error | correction | analysis |     |     |     |     |     |
| to incorporate | additional |          | constraints |     | is an advantage |          | of our |          |            |          |     |     |     |     |     |
approach. Theoretical false positive rates can be easily estimated.
Somecodewords,despitesatisfyingtheHammingdistance Assumethatafalsequadisidentifiedandthatthebitpattern
constraint, are poor choices. For example, a code word israndom.Theprobabilityofafalsepositiveisthefractionof
consisting of all zeros would result in a tag that looks like codewords which are accepted as valid tags versus the total
a single black square. Such simple geometric patterns com- number of possible codewords, 2n. More aggressive error
monly occur in natural scenes, resulting in false positives. correction increases this rate, since it increases the number

ofcodewordsthatareaccepted.Thisunavoidableincreasein
errorrateisillustratedforthe36h10and36h15codesbelow:
|     | Bits | corrected | 36h10 | FP (%)   | 36h15 | FP       | (%) |                 |            |     |            |      |               |     |           |
| --- | ---- | --------- | ----- | -------- | ----- | -------- | --- | --------------- | ---------- | --- | ---------- | ---- | ------------- | --- | --------- |
|     |      | 0         |       | 0.000001 |       | 0.000000 |     |                 |            |     |            |      |               |     |           |
|     |      | 1         |       | 0.000041 |       | 0.000002 |     |                 |            |     |            |      |               |     |           |
|     |      | 2         |       | 0.000744 |       | 0.000029 |     |                 |            |     |            |      |               |     |           |
|     |      | 3         |       | 0.008714 |       | 0.000341 |     |                 |            |     |            |      |               |     |           |
|     |      | 4         |       | 0.074459 |       | 0.002912 |     |                 |            |     |            |      |               |     |           |
|     |      | 5         |       | 0.495232 |       | 0.019370 |     |                 |            |     |            |      |               |     |           |
|     |      | 6         |       | N/A      |       | 0.104403 |     |                 |            |     |            |      |               |     |           |
|     |      | 7         |       | N/A      |       | 0.468827 |     |                 |            |     |            |      |               |     |           |
|     |      |           |       |          |       |          |     | Fig. 5. Hamming | distances. |     | Good codes | have | large Hamming |     | distances |
Of course, the better performance of the 36h15 encoding between valid codewords. Shown above are the Hamming distances for
comes at a price: there are only 27 distinguishable code- several contemporary systems. Note that our coding scheme guarantees a
minimumHammingdistancebyconstruction,whereasothersystemshave
words, as opposed to 36h10’s 2221 distinguishable code- someverysimilarcodewordswhichleadstohigherinter-tagconfusionrates.
words.
| Our | coding | scheme | is  | significantly | stronger | than | previous |     |     |     |     |     |     |     |     |
| --- | ------ | ------ | --- | ------------- | -------- | ---- | -------- | --- | --- | --- | --- | --- | --- | --- | --- |
schemes, including that used by ARTag and both systems imagecorpusfromtheLabelMedataset[26],whichcontains
usedbyARToolkitPlus:ourcodingsystemachievesagreater 180,829 images from a wide variety of indoor and outdoor
minimumHammingdistancebetweenallpairsofcodewords environments.Sincenoneoftheseimagescontainoneofour
while encoding a larger number of distinguishable ids. This tags, we can measure the false positive rate of our coding
| improvement |          | in minimum |        | Hamming            | distance |      | is illustrated |         |          |               |     |     |     |     |     |
| ----------- | -------- | ---------- | ------ | ------------------ | -------- | ---- | -------------- | ------- | -------- | ------------- | --- | --- | --- | --- | --- |
|             |          |            |        |                    |          |      |                | systems | by using | these images. |     |     |     |     |     |
| in Fig.     | 5        | and in the | table  | below:             |          |      |                |         |          |               |     |     |     |     |     |
| Encoding    |          | Scheme     | Length | Unique             | codes    |      | Min. Hamming   |         |          |               |     |     |     |     |     |
| ARToolkit+  |          | (simple)   |        | 36                 | 512      |      | 4              |         |          |               |     |     |     |     |     |
| ARToolkit+  |          | (BCH)      |        | 36                 | 4096     |      | 2              |         |          |               |     |     |     |     |     |
|             | ARTag    |            |        | 36                 | 2046     |      | 4              |         |          |               |     |     |     |     |     |
|             | Proposed | (36h9)     |        | 36                 | 4164     |      | 9              |         |          |               |     |     |     |     |     |
|             | Proposed | (36h10)    |        | 36                 | 2221     |      | 10             |         |          |               |     |     |     |     |     |
| In          | order    | to decode  | a      | possibly-corrupted |          | code | word, the      |         |          |               |     |     |     |     |     |
Hamming distance between the code word and each valid Fig.6. Empiricalfalsepositivesversustagcomplexity.Ourtheoreticalerror
ratesassumethatallcodewordsareequallylikelytooccurbychanceina
| code | word | in the | code book | is computed. |     | If the | best match |     |     |     |     |     |     |     |     |
| ---- | ---- | ------ | --------- | ------------ | --- | ------ | ---------- | --- | --- | --- | --- | --- | --- | --- | --- |
real-worldenvironment.Ourhypothesisisthatreal-worldenvironmentsare
has a Hamming distance less than the user-specified thresh- biasedtowardscodeswhichhavealowrectanglecoveringcomplexityand
old,adetection isreported. Byspecifying thisthreshold, the that by selecting codewords that have high rectangle covering complexity,
|      |         |            |     |          |         |       |           | we can decrease | false | positive | rates. | This hypothesis | is  | validated | by the |
| ---- | ------- | ---------- | --- | -------- | ------- | ----- | --------- | --------------- | ----- | -------- | ------ | --------------- | --- | --------- | ------ |
| user | is able | to control | the | tradeoff | between | false | positives |                 |       |          |        |                 |     |           |        |
graphabove,whichshowsempiricalfalsepositiveratesfromtheLabelMe
| and | false | negatives. |     |     |     |     |     |     |     |     |     |     |     |     |     |
| --- | ----- | ---------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
dataset(solidlines)forrectanglecoveringcomplexitiesfromc=2toc=10.
Adisadvantageofourmethodisthatthisdecodingprocess Atcomplexitiesofc=9andc=10,thefalsepositiveratedropsbelowtherate
|       |        |      |        |         |               |     |             | predicted | by the pessimistic | model | that | real-world | payloads | are randomly |     |
| ----- | ------ | ---- | ------ | ------- | ------------- | --- | ----------- | --------- | ------------------ | ----- | ---- | ---------- | -------- | ------------ | --- |
| takes | linear | time | in the | size of | the codebook, |     | since every |           |                    |       |      |            |          |              |     |
distributed.
validcodewordmustbeconsidered.However,thecoefficient
is so small that the computational complexity is negligible Evaluationofcomplexityheuristic: Wefirstwishtoevalu-
| in comparison |         | to     | the other | image  | processing | steps. |            |                    |     |           |            |          |           |            |        |
| ------------- | ------- | ------ | --------- | ------ | ---------- | ------ | ---------- | ------------------ | --- | --------- | ---------- | -------- | --------- | ---------- | ------ |
|               |         |        |           |        |            |        |            | ate our hypothesis |     | that the  | false      | positive | rate can  | be reduced |        |
| For           | a given | coding | scheme,   | larger | tags       | (i.e., | those with |                    |     |           |            |          |           |            |        |
|               |         |        |           |        |            |        |            | by imposing        | our | geometric | complexity |          | heuristic | to         | reject |
36 bits versus 25 bits) have dramatically better coding candidate codewords. To do this, we generated ten variants
performancethansmallertags,althoughthiscomesataprice.
ofthe25h9familywithminimumcomplexitiesrangingfrom
All other things being equal, the range at which a given 1 to 10. In Fig. 6, the false positive rate is given for each
cameracanreada36bittagwillbeshorterthantherangeat
|     |     |     |     |     |     |     |     | of these | complexities | as  | a function | of  | the maximum | number |     |
| --- | --- | --- | --- | --- | --- | --- | --- | -------- | ------------ | --- | ---------- | --- | ----------- | ------ | --- |
whichthesamecameracanreada16or25bittag.However,
ofbiterrorscorrected.Alsodisplayedisthetheoreticalfalse
thebenefitinrangeforsmallertagsisquitemodestduetothe positive rate which was based on the assumption that data
4pixeloverheadoftheborders;onlya25%improvementin
|     |     |     |     |     |     |     |     | payloads | are randomly | distributed. |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | -------- | ------------ | ------------ | --- | --- | --- | --- | --- |
detection range can be expected by using 16 bit tags instead Fig. 6 clearly demonstrates that our heuristic is effective
| of           | 36 bit | tags. Thus, | it      | is only in | the most      | range-sensitive |     |             |     |                |     |       |                |      |     |
| ------------ | ------ | ----------- | ------- | ---------- | ------------- | --------------- | --- | ----------- | --- | -------------- | --- | ----- | -------------- | ---- | --- |
|              |        |             |         |            |               |                 |     | in reducing | the | false positive |     | rate. | Interestingly, | once | the |
| applications |        | where       | smaller | tags are   | advantageous. |                 |     |             |     |                |     |       |                |      |     |
complexityexceeds8,theperformanceisactuallybetterthan
|     |     |     |                     |     |     |     |     | predicted  | by theory. |       |        |          |     |              |     |
| --- | --- | --- | ------------------- | --- | --- | --- | --- | ---------- | ---------- | ----- | ------ | -------- | --- | ------------ | --- |
|     |     | VI. | EXPERIMENTALRESULTS |     |     |     |     |            |            |       |        |          |     |              |     |
|     |     |     |                     |     |     |     |     | Comparison | to         | other | coding | schemes: | We  | next compare |     |
A. Empirical Experiments the false positive of our rate of our coding systems to those
A key question we wish to answer is whether our analyt- used by ARToolkitPlus and ARTag.
ical predictions regarding false positive rates holds in real- Using the same real-world imagery dataset, we plot the
world imagery. To answer this question, we used a standard empirical false positive rates for five codes in Fig. 7.

Fig.7. Empiricalfalsepositives.Thebestperformingmethods,interms
of the rate of false positives on the LabelMe dataset, are 36h15 and
ARToolkitPlus-Simple. These two coding families also have the fewest
numberofdistinguishablecodes,whichgivesthemanadvantage.Theother
three systems have approximately comparable numbers of distinguishable
codes;ARToolkitPlus-BCHperformsverypoorly;ARTagdoesmuchbetter,
andourproposed36h10encodingdoesevenbetter.
Fig. 9. Orientation accuracy. Using our ray-tracing simulator (which
ARToolkitPlus’s BCH coding scheme has the highest false
provides ground truth), we evaluated the accuracy of two tag detector’s
positive rate, followed by ARTag. Our 36h10 encoding, localization accuracy. We fixed the range to the tag and varied the angle
generatedwithaminimumcomplexityof10,performsbetter between the tag’s normal and the camera direction. For both systems,
the performance in localization accuracy and in success rate worsens as
than both of these systems. This is a central result of this
the tag rotates away from the camera. However, the proposed system has
paper. dramatically lower localization error and is able to detect targets more
reliably.
and the vector to the camera. When φ is 0, the target is
facing directly towards the target; as φ approaches π/2,
the target rotates out of view and we expect performance
to decrease. We measure performance in terms of both the
localization accuracy and detection rate. In Fig. 9, we see
thatourdetectorsignificantlyoutperformstheARToolkitPlus
detector: not only are the orientation and distance estimates
more accurate, but it can detect tags over a larger range of
φ.
The complementary experiment is to hold φ = 0 and
Fig.8. Examplesyntheticimage.Wegeneratedray-tracedimagesinorder
to vary the distance. We expect that as distance increases,
tocreateground-trutheddatasetsforourevaluation.Inthisexample,thetag
is 10m from the camera, and its normal vector points 30.3 degrees away accuracy will decrease. In Fig. 10, we see that our detector
fromthecamera. works reliably to 50 m, while the ARToolkitPlus detector’s
detection rate drops to under 50% at around 25 m. In
The plot shows data for two additional schemes: ARTP-
addition, our detector provides significantly more accurate
Simple performs about the same as our 36h10 encoding, but
localization results.
because its tag family has one quarter as many distinguish-
Naturally, real-world performance of the system will be
abletags,itsfalsepositiverateiscorrespondinglylower.For
lowerthanthesesyntheticexperimentsduetonoise,lighting
comparison purposes, we also include the false positive rate
variation, and other non-idealities (such as lens distortion or
for 36h15 family with only 27 distinguishable codewords.
tag non-planarity). Still, the real-world performance of our
As expected, it has an exceptionally low false positive rate.
system has been very good.
While our methods are generally more computationally
B. Localization Accuracy
expensive than those used by ARToolkitPlus, our Java im-
To evaluate the localization accuracy of the detector, we plementation runs at interactive rates (30 fps) on VGA
usedaraytracertogenerateimageswithknowngroundtruth resolution images (Intel Core2 CPU at 2.6GHz). Higher
(seeFig.8foranexample).Thetruelocationandorientation resolutions signficantly impact runtime due to the graph-
ofthetagswasvariedrandomlyandcomparedtothedetected based clustering. We expect significant speedups by making
position.Imagesweregeneratedataresolutionof400×400 use of SIMD optimizations and accelerated image procesing
with a pinhole lens and focal length of 400 pixels. libraries in our ongoing C port.
The main factor in localization accuracy is the size of the
target,whichisaffectedbyboththedistanceandorientation
VII. CONCLUSION
of the tag. To decouple these factors, we conducted two We have described a visual fiducial system that signifi-
experiments. The first experiment measured the orientation cantlyimprovesuponpreviousmethods.Wedescribedanew
accuracy of targets while fixing the distance. The critical approach for detecting edges using a graph-based clustering
parameter is the angle φ between the target’s normal vector method along with a coding system that is demonstrably

|     |     |     |     |     |     |     |     | [10] T. Lochmatter, |     | P. Roduit, | C. Cianci,   | N.  | Correll,    | J. Jacot, and |
| --- | --- | --- | --- | --- | --- | --- | --- | ------------------- | --- | ---------- | ------------ | --- | ----------- | ------------- |
|     |     |     |     |     |     |     |     | A. Martinoli,       |     | “SwisTrack | - A Flexible |     | Open Source | Tracking      |
SoftwareforMulti-AgentSystems,”inProceedingsoftheIEEE/RSJ
|     |     |     |     |     |     |     |     | 2008  | International | Conference  | on             | Intelligent | Robots    | and Systems |
| --- | --- | --- | --- | --- | --- | --- | --- | ----- | ------------- | ----------- | -------------- | ----------- | --------- | ----------- |
|     |     |     |     |     |     |     |     | (IROS | 2008).        | IEEE, 2008, | pp. 4004–4010. |             | [Online]. | Available:  |
http://iros2008.inria.fr/
|     |     |     |     |     |     |     |     | [11] H.KatoandM.Billinghurst,“Markertrackingandhmdcalibrationfor |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | ---------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
avideo-basedaugmentedrealityconferencingsystem,”inIWAR’99:
|     |     |     |     |     |     |     |     | Proceedings       | of  | the 2nd IEEE                           | and ACM | International |     | Workshop on |
| --- | --- | --- | --- | --- | --- | --- | --- | ----------------- | --- | -------------------------------------- | ------- | ------------- | --- | ----------- |
|     |     |     |     |     |     |     |     | AugmentedReality. |     | Washington,DC,USA:IEEEComputerSociety, |         |               |     |             |
1999,p.85.
|     |     |     |     |     |     |     |     | [12] D.WagnerandD.Schmalstieg,“Makingaugmentedrealitypractical |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | -------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
onmobilephones,part1,”IEEEComputerGraphicsandApplications,
vol.29,pp.12–15,2009.
|     |     |     |     |     |     |     |     | [13] S.-w.Lee,D.-c.Kim,D.-y.Kim,andT.-d.Han,“Tagdetectionalgo- |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | -------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
rithmforimprovingtheinstabilityproblemofanaugmentedreality,”
|     |     |     |     |     |     |     |     | in ISMAR                             | ’06: | Proceedings | of the 5th | IEEE | and ACM            | International |
| --- | --- | --- | --- | --- | --- | --- | --- | ------------------------------------ | ---- | ----------- | ---------- | ---- | ------------------ | ------------- |
|     |     |     |     |     |     |     |     | SymposiumonMixedandAugmentedReality. |      |             |            |      | Washington,DC,USA: |               |
IEEEComputerSociety,2006,pp.257–258.
Fig.10. Distanceaccuracy.Inordertoevaluatetheaccuracyofdistance
|     |     |     |     |     |     |     |     | [14] D.ParikhandG.Jancke,“Localizationandsegmentationofa2dhigh |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | -------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
estimationtothetarget,weagainusedground-truthedsimulationdata.For
capacitycolorbarcode,”inWACV’08:Proceedingsofthe2008IEEE
thisexperiment,wefixedthetargetsothatitfacedthecamerabutvariedthe
|     |     |     |     |     |     |     |     | Workshop | on Applications |     | of Computer | Vision. | Washington, | DC, |
| --- | --- | --- | --- | --- | --- | --- | --- | -------- | --------------- | --- | ----------- | ------- | ----------- | --- |
distancebetweenthetagandthecamera.Ourproposedmethodsignificantly USA:IEEEComputerSociety,2008,pp.1–6.
| outperforms | ARToolkitPlus, | providing |     | more accurate | range | estimates, | and |            |         |           |           |                |     |              |
| ----------- | -------------- | --------- | --- | ------------- | ----- | ---------- | --- | ---------- | ------- | --------- | --------- | -------------- | --- | ------------ |
|             |                |           |     |               |       |            |     | [15] P. C. | Santos, | A. Stork, | A. Buaes, | C. E. Pereira, | and | J. Jorge, “A |
overtwicetheworkingdetectionrange.
real-timelow-costmarker-basedmultiplecameratrackingsolutionfor
virtualrealityapplications,”January2009.
|     |     |     |     |     |     |     |     | [16] A.    | Mohan, | G. Woo,  | S. Hiura,     |        | Q. Smithwick, | and        |
| --- | --- | --- | --- | --- | --- | --- | --- | ---------- | ------ | -------- | ------------- | ------ | ------------- | ---------- |
|     |     |     |     |     |     |     |     | R. Raskar, |        | “Bokode: | imperceptible | visual | tags          | for camera |
strongerthanpreviousmethods.Wehavealsodescribedaset
|     |     |     |     |     |     |     |     | based | interaction | from | a distance,” |     | ACM Trans. | Graph., |
| --- | --- | --- | --- | --- | --- | --- | --- | ----- | ----------- | ---- | ------------ | --- | ---------- | ------- |
of benchmarks that we hope will make it easier to evaluate vol. 28, pp. 98:1–98:8, July 2009. [Online]. Available:
othermethodsinthefuture.Incontrasttoothersystems(with http://portal.acm.org/citation.cfm?id=1531326.1531404
the notable exception of ARToolkit), our implementation is [17] J.McLurkin,“Analysisandimplementationofdistributedalgorithms
|     |     |     |     |     |     |     |     | for Multi-Robot |     | systems,” | Ph.D. thesis, | Massachusetts |     | Institute of |
| --- | --- | --- | --- | --- | --- | --- | --- | --------------- | --- | --------- | ------------- | ------------- | --- | ------------ |
fully open. Our source code and benchmarking software are Technology,2008.
freely available: [18] Y.Yamamoto,P.Pirjanian,M.Munich,E.Dibernardo,L.Goncalves,
J.Ostrowski,andN.Karlsson,“Opticalsensingforrobotperception
http://april.eecs.umich.edu/ andlocalization,”in2005IEEEWorkshoponAdvancedRoboticsand
itsSocialImpacts,2005,pp.14–17.
|     |     |     |     |     |     |     |     | [19] P.F.FelzenszwalbandD.P.Huttenlocher,“Efficientgraph-basedim- |     |     |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | ----------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
REFERENCES
agesegmentation,”InternationalJournalofComputerVision,vol.59,
no.2,pp.167–181,2004.
|           |            |       |           |              |              |     |        | [20] R. L. | RivestandC. | E. Leiserson, | Introductionto |     | Algorithms. | New |
| --------- | ---------- | ----- | --------- | ------------ | ------------ | --- | ------ | ---------- | ----------- | ------------- | -------------- | --- | ----------- | --- |
| [1] C.-H. | Chu, D.-N. | Yang, | and M.-S. | Chen, “Image | stablization |     | for 2d |            |             |               |                |     |             |     |
York,NY,USA:McGraw-Hill,Inc.,1990.
barcodeinhandhelddevices,”inMULTIMEDIA’07:Proceedingsof
the 15th international conference on Multimedia. New York, NY, [21] D. G. Lowe, “Distinctive image features from scale-invariant key-
USA:ACM,2007,pp.697–706. points,”InternationalJournalofComputerVision,vol.60,no.2,pp.
[2] D.Wagner,G.Reitmayr,A.Mulloni,T.Drummond,andD.Schmal- 91–110,November2004.
|     |     |     |     |     |     |     |     | [22] R. Hartley | and | A. Zisserman, | Multiple | View | Geometry | in Computer |
| --- | --- | --- | --- | --- | --- | --- | --- | --------------- | --- | ------------- | -------- | ---- | -------- | ----------- |
stieg,“Posetrackingfromnaturalfeaturesonmobilephones,”inIS-
|     |     |     |     |     |     |     |     | Vision,2nded. |     | CambridgeUniversityPress,2004. |     |     |     |     |
| --- | --- | --- | --- | --- | --- | --- | --- | ------------- | --- | ------------------------------ | --- | --- | --- | --- |
MAR’08:Proceedingsofthe7thIEEE/ACMInternationalSymposium
on Mixed and Augmented Reality. Washington, DC, USA: IEEE [23] K. Shoemake and T. Duff, “Matrix animation and polar decomposi-
ComputerSociety,2008,pp.125–134. tion,” in In Proceedings of the conference on Graphics interface 92.
[3] M.Fiala,“ARTag,afiducialmarkersystemusingdigitaltechniques,” MorganKaufmannPublishersInc,1992,pp.258–264.
|         |      |             |     |               |          |     |         | [24] A.Trachtenberg,A.T.M.S,E.Vardy,andC.L.Liu,“Computational |     |     |     |     |     |     |
| ------- | ---- | ----------- | --- | ------------- | -------- | --- | ------- | ------------------------------------------------------------- | --- | --- | --- | --- | --- | --- |
| in CVPR | ’05: | Proceedings | of  | the 2005 IEEE | Computer |     | Society |                                                               |     |     |     |     |     |     |
methodsincodingtheory,”Tech.Rep.,1996.
ConferenceonComputerVisionandPatternRecognition(CVPR’05)
-Volume2. Washington,DC,USA:IEEEComputerSociety,2005, [25] R.A.BrualdiandV.S.Pless,“Greedycodes,”J.Comb.TheorySer.
| pp.590–596. |     |     |     |     |     |     |     | A,vol.64,no.1,pp.10–30,1993. |     |     |     |     |     |     |
| ----------- | --- | --- | --- | --- | --- | --- | --- | ---------------------------- | --- | --- | --- | --- | --- | --- |
[4] A. C. Sementille, L. E. Lourenc¸o, J. R. F. Brega, and I. Rodello, [26] B. C. Russell, A. Torralba, K. P. Murphy, and W. T. Freeman,
“Labelme:Adatabaseandweb-basedtoolforimageannotation,”Tech.
| “A motion   | capture | system   | using | passive markers,” |               | in VRCAI   | ’04: |      |                        |     |               |     |           |             |
| ----------- | ------- | -------- | ----- | ----------------- | ------------- | ---------- | ---- | ---- | ---------------------- | --- | ------------- | --- | --------- | ----------- |
|             |         |          |       |                   |               |            |      | Rep. | MIT-CSAIL-TR-2005-056, |     | Massachusetts |     | Institute | of Technol- |
| Proceedings | of      | the 2004 | ACM   | SIGGRAPH          | international | conference |      |      |                        |     |               |     |           |             |
ogy,Tech.Rep.,2005.
| onVirtualRealitycontinuumanditsapplicationsinindustry. |     |     |     |     |     |     | New |     |     |     |     |     |     |     |
| ------------------------------------------------------ | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
York,NY,USA:ACM,2004,pp.440–447.
[5] J.Sattar,P.Gigue`re,andG.Dudek,“Sensor-basedbehaviorcontrolfor
anautonomousunderwatervehicle,”Int.J.Rob.Res.,vol.28,no.6,
pp.701–713,2009.
[6] M.Fiala,“Arobotcontrolandaugmentedrealityinterfaceformultiple
robots,”inCRV’09:Proceedingsofthe2009CanadianConferenceon
| ComputerandRobotVision. |     |     | Washington,DC,USA:IEEEComputer |     |     |     |     |     |     |     |     |     |     |     |
| ----------------------- | --- | --- | ------------------------------ | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
Society,2009,pp.31–36.
[7] ——,“Visionguidedcontrolofmultiplerobots,”ComputerandRobot
Vision,CanadianConference,vol.0,pp.241–246,2004.
[8] U.Frese,“Deutscheszentrumfu¨rluft-undraumfahrt(DLR)dataset,”
2003.
| [9] J. Sattar, | E. Bourque, |     | P. Giguere, | and G. | Dudek, | “Fourier | tags: |     |     |     |     |     |     |     |
| -------------- | ----------- | --- | ----------- | ------ | ------ | -------- | ----- | --- | --- | --- | --- | --- | --- | --- |
Smoothlydegradablefiducialmarkersforuseinhuman-robotinterac-
tion,”ComputerandRobotVision,CanadianConference,vol.0,pp.
165–174,2007.