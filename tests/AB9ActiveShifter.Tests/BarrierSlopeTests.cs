using System;
using AB9ActiveShifter.Core;
using Xunit;

namespace AB9ActiveShifter.Tests
{
    /// <summary>
    /// A barrier between two columns may not be steeper than the walls. Traced on 6+R with the
    /// base's spring at 50: the spring rests the lever at the axis centre, which in a four-column
    /// gate is a barrier's crest, and at 40.2% force and a width of 400 the lever bounced 3300 to
    /// 4200 counts at about 15 Hz and never settled (trace-20261116-174017, -180817). The lever
    /// model here is the one LateralBounceTests fitted to this rig; it reproduces that bounce at
    /// the dialled width and shows it gone at the limited one.
    /// </summary>
    public class BarrierSlopeTests
    {
        private const int Centre = GateGeometry.AxisCenter;

        /// <summary>The profile the traces were taken on, as far as neutral's lateral field goes.</summary>
        private static EngineConfig Traced()
        {
            return new EngineConfig
            {
                OverallGainPct = 100,
                PolarityConfirmed = true,
                Pattern = GatePattern.H6R,
                PatternWidthPct = 60,
                SlotHalfWidth = 2400,
                ColumnPinForcePct = 80,
                ChannelWallForcePct = 100,
                ChannelGuideForcePct = 8.96,
                ColumnDetentForcePct = 0,
                PatternEdgeForcePct = 100,
                MouthShape = SlotMouthShape.Angled,
                MouthDepth = 12000,
                MouthOpenPct = 39,
                ChannelHalfEnter = 3268,
                ChannelHalfExit = 4051,
                ChannelFreeDepth = 2668,
                EngageDepth = 20852,
                ReleaseDepth = 23852,
                WallRamp = 3285,
                WallBlend = 3106,
                WallAttackMs = 15,
                WallYieldPct = 45.28,
                YieldRecoveryMs = 20,
                YieldVelocityDeadband = 10000,
                YieldVelocityBlend = 12000,
                WallFrictionPct = 6,
                DampingPct = 0,
                BarrierForcePct = 40.2,
                BarrierWidth = 400,
                BaseSpringPct = 50,
                LockoutForcePct = 90,
                LockoutHalfWidth = 6000
            };
        }

        /// <summary>
        /// Releases the lever in neutral a little off the centre with the base's spring pulling it
        /// back, and returns the peak-to-peak lateral swing over the fourth second.
        /// </summary>
        private static int Swing(EngineConfig cfg, int delayTicks)
        {
            // Counts/s^2 per DI unit and per count from centre, and the base's own damping - the
            // constants LateralBounceTests fitted to this rig at AB9 Spring 50.
            const double PerDi = 5600;
            const double Spring = 1600;
            const double Damping = 15;

            var composer = new ForceComposer(cfg.BuildGeometry(), cfg);
            var estimator = new VelocityEstimator();
            var inFlight = new int[delayTicks + 1];

            double x = Centre - 1000, v = 100000;
            double min = double.MaxValue, max = double.MinValue;

            for (int tick = 0; tick < 4000; tick++)
            {
                int reported = (int)Math.Round(x);
                estimator.Update(reported, Centre, tick);
                ForceFrame f = composer.Compose(GateState.Neutral, Column.None, ShiftDir.None,
                    reported, Centre, estimator.X, estimator.Y, 1.0);

                Array.Copy(inFlight, 1, inFlight, 0, delayTicks);
                inFlight[delayTicks] = f.ConstantX;
                int acting = inFlight[0];

                for (int step = 0; step < 10; step++)
                {
                    double a = PerDi * acting + Spring * (Centre - x) - Damping * v;
                    v += a * 0.0001;
                    x += v * 0.0001;
                }

                if (tick >= 3000)
                {
                    min = Math.Min(min, x);
                    max = Math.Max(max, x);
                }
            }

            return (int)(max - min);
        }

        [Theory]
        [InlineData(3)]
        [InlineData(4)]
        public void ALeverRestingOnTheTracedBarrierSettles(int delayTicks)
        {
            // 3577 and 4226 counts of swing at the dialled width of 400; the trace swung 3300 to
            // 4200. With the width held at the walls' stiffness the model comes to rest.
            int swing = Swing(Traced(), delayTicks);
            Assert.True(swing < 500, "at " + delayTicks + " ms the lever still swings " + swing + " counts");
        }

        [Fact]
        public void TheTracedBarrierIsRenderedAtTheWallsStiffness()
        {
            EngineConfig cfg = Traced();
            var composer = new ForceComposer(cfg.BuildGeometry(), cfg);

            // exp(0.5) * 4020 * 3285 / 8000 = 2721.6
            Assert.Equal(2722, composer.BarrierWidth);
            Assert.Equal(4020, composer.BarrierForce);   // strength is untouched: there is room
        }

        [Fact]
        public void NoCountOfTheBarrierIsSteeperThanTheWall()
        {
            EngineConfig cfg = Traced();
            GateGeometry geo = cfg.BuildGeometry();
            var composer = new ForceComposer(geo, cfg);
            double wall = 8000.0 / cfg.WallRamp;   // pin force over the wall's bite

            int crest = geo.BarrierCentre(1);      // the ordinary gap between the two middle columns
            double steepest = 0;
            for (int x = crest - 6000; x < crest + 6000; x += 20)
            {
                // Over 100 counts, so integer rounding of the force does not read as slope.
                double slope = Math.Abs(composer.BarrierForceIn(x + 100, Centre) - composer.BarrierForceIn(x, Centre)) / 100.0;
                steepest = Math.Max(steepest, slope);
            }

            Assert.True(steepest <= wall * 1.02, "steepest " + steepest + " against a wall of " + wall);
            Assert.True(steepest >= wall * 0.9, "the limit should bind exactly, not over-soften: " + steepest);
        }

        [Fact]
        public void ABarrierAlreadyGentleEnoughIsNotTouched()
        {
            // The default tune and every shipped preset must render exactly what they always did.
            var defaults = new EngineConfig { OverallGainPct = 100, PolarityConfirmed = true };
            var plain = new ForceComposer(defaults.BuildGeometry(), defaults);
            Assert.Equal(defaults.BarrierWidth, plain.BarrierWidth);
            Assert.Equal((int)Math.Round(defaults.BarrierForcePct * 100), plain.BarrierForce);

            foreach (ShifterProfile p in DefaultProfiles.Create().Profiles)
            {
                EngineConfig cfg = p.Settings.ToEngineConfig();
                var composer = new ForceComposer(cfg.BuildGeometry(), cfg);
                Assert.True(composer.BarrierWidth == Math.Max(1, cfg.BarrierWidth), p.Name + " had its barrier widened");
                Assert.False(p.Settings.BarrierIsWidened, p.Name);
            }
        }

        [Fact]
        public void WithNoRoomToWidenTheStrengthComesDownInstead()
        {
            // A narrow four-column gate with soft walls: the floor is wider than half the gap, so
            // the hump keeps to the gap and gives up strength - gentler, never steeper.
            EngineConfig cfg = Traced();
            cfg.PatternWidthPct = 25;
            cfg.BarrierForcePct = 100;
            GateGeometry geo = cfg.BuildGeometry();
            var composer = new ForceComposer(geo, cfg);

            Assert.Equal(geo.ColumnSpacing / 2, composer.BarrierWidth);
            Assert.True(composer.BarrierForce < 10000);
            double crestSlope = ForceComposer.HumpCrestSlope * composer.BarrierForce / composer.BarrierWidth;
            Assert.True(crestSlope <= 8000.0 / cfg.WallRamp + 0.001, "crest slope " + crestSlope);
        }

        [Fact]
        public void AWidthAlreadyWiderThanTheGapIsLeftAsDialled()
        {
            // Someone who asked for a very broad rise gets it; the limit only ever widens or softens.
            EngineConfig cfg = Traced();
            cfg.BarrierWidth = 8000;
            var composer = new ForceComposer(cfg.BuildGeometry(), cfg);

            Assert.Equal(8000, composer.BarrierWidth);
            Assert.Equal(4020, composer.BarrierForce);
        }

        [Fact]
        public void TheFloorIsARatioSoPercentagesAndForcesAgree()
        {
            Assert.Equal(ForceComposer.BarrierWidthFloor(4020, 8000, 3285), ForceComposer.BarrierWidthFloor(40.2, 80, 3285));
            Assert.Equal(1, ForceComposer.BarrierWidthFloor(0, 80, 3285));
            Assert.Equal(1, ForceComposer.BarrierWidthFloor(40, 0, 3285));
        }

        [Fact]
        public void TheSettingsPageSaysWhenAClickIsBeingWidened()
        {
            var s = new ShifterSettings
            {
                BarrierForcePct = 40.2,
                BarrierWidth = 400,
                ColumnPinForcePct = 80,
                WallRamp = 3285
            };
            Assert.True(s.BarrierIsWidened);
            Assert.Contains("2722", s.BarrierWidenedNote);

            int raised = 0;
            s.PropertyChanged += (o, e) => { if (e.PropertyName == "BarrierIsWidened") raised++; };
            s.BarrierForcePct = 5;

            Assert.Equal(1, raised);
            Assert.False(s.BarrierIsWidened);
            Assert.Equal("", s.BarrierWidenedNote);
            Assert.False(ProfileTransfer.IsTuning("BarrierIsWidened") && ProfileTransfer.IsMachineFact("BarrierIsWidened"));
        }
    }
}
